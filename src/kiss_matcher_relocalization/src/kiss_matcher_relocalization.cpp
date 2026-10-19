#include "kiss_matcher_relocalization/kiss_matcher_relocalization.hpp"

#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>

#include "pcl/common/transforms.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace kiss_matcher_relocalization
{

KissMatcherRelocalizationNode::KissMatcherRelocalizationNode(
  const rclcpp::NodeOptions & options)
: Node("kiss_matcher_relocalization", options),
  startup_reloc_done_(false),
  is_relocating_(false),
  current_failures_(0),
  accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
  target_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
{
  // Parameters
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("input_cloud_topic", "/cloud_filtered");
  this->declare_parameter("failures_topic", "/gicp/consecutive_failures");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("robot_base_frame", "base_link");
  this->declare_parameter("voxel_resolution", 0.5f);
  this->declare_parameter("accumulate_duration", 3.0);
  this->declare_parameter("failure_threshold", 20);
  this->declare_parameter("cooldown_duration", 10.0);
  this->declare_parameter("use_quatro", true);
  this->declare_parameter("num_inliers_threshold", 15);
  this->declare_parameter("preset_x", 0.0);
  this->declare_parameter("preset_y", 0.0);
  this->declare_parameter("preset_z", 0.0);
  this->declare_parameter("preset_yaw", 0.0);

  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("failures_topic", failures_topic_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("voxel_resolution", voxel_resolution_);
  this->get_parameter("accumulate_duration", accumulate_duration_);
  this->get_parameter("failure_threshold", failure_threshold_);
  this->get_parameter("cooldown_duration", cooldown_duration_);
  this->get_parameter("use_quatro", use_quatro_);
  this->get_parameter("num_inliers_threshold", num_inliers_threshold_);
  this->get_parameter("preset_x", preset_x_);
  this->get_parameter("preset_y", preset_y_);
  this->get_parameter("preset_z", preset_z_);
  this->get_parameter("preset_yaw", preset_yaw_);

  RCLCPP_INFO(
    this->get_logger(),
    "Parameters: voxel_res=%.2f, accumulate=%.1fs, failure_thr=%d, "
    "cooldown=%.1fs, use_quatro=%s, inliers_thr=%d, preset=[%.2f,%.2f,%.2f,%.2f]",
    voxel_resolution_, accumulate_duration_, failure_threshold_,
    cooldown_duration_, use_quatro_ ? "true" : "false", num_inliers_threshold_,
    preset_x_, preset_y_, preset_z_, preset_yaw_);

  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Load prior map
  if (!prior_pcd_file_.empty()) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(prior_pcd_file_, *target_cloud_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load prior PCD: %s", prior_pcd_file_.c_str());
    } else {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*target_cloud_, *target_cloud_, indices);
      RCLCPP_INFO(
        this->get_logger(), "Loaded prior map: %zu points",
        target_cloud_->points.size());
      // 【新增】如果 preset 未设置(全0)，自动使用 PCD 的 VIEWPOINT 作为预设原点
      bool preset_unset = (std::abs(preset_x_) < 0.001 && std::abs(preset_y_) < 0.001 &&
                           std::abs(preset_yaw_) < 0.001);
      if (preset_unset) {
        preset_x_ = target_cloud_->sensor_origin_.x();
        preset_y_ = target_cloud_->sensor_origin_.y();
        preset_z_ = target_cloud_->sensor_origin_.z();
        Eigen::Quaterniond q(target_cloud_->sensor_orientation_.w(),
                             target_cloud_->sensor_orientation_.x(),
                             target_cloud_->sensor_orientation_.y(),
                             target_cloud_->sensor_orientation_.z());
        Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
        preset_yaw_ = rpy.z();
        RCLCPP_INFO(this->get_logger(),
          "Auto-set preset from PCD viewpoint: [%.3f, %.3f, %.3f, yaw=%.3f rad]",
          preset_x_, preset_y_, preset_z_, preset_yaw_);
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "prior_pcd_file is empty!");
  }

  // Publishers
  initial_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  status_pub_ =
    this->create_publisher<std_msgs::msg::Int32>("km_reloc_status", 10);

  // Subscribers
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&KissMatcherRelocalizationNode::cloudCallback, this, std::placeholders::_1));
  failures_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    failures_topic_, 10,
    std::bind(&KissMatcherRelocalizationNode::failuresCallback, this, std::placeholders::_1));

  // Timer: 1 Hz check
  check_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&KissMatcherRelocalizationNode::checkAndRelocate, this));

  startup_time_ = this->now();
  last_reloc_time_ = this->now();

  RCLCPP_INFO(
    this->get_logger(),
    "KISS-Matcher Relocalization Node started. Waiting %.1fs for point cloud accumulation...",
    accumulate_duration_);
}

void KissMatcherRelocalizationNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  *accumulated_cloud_ += *scan;
}

void KissMatcherRelocalizationNode::failuresCallback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  current_failures_ = msg->data;
}

void KissMatcherRelocalizationNode::checkAndRelocate()
{
  const auto now = this->now();
  const double elapsed = (now - startup_time_).seconds();

  // Startup phase: wait for accumulation, then run once
  if (!startup_reloc_done_) {
    if (elapsed < accumulate_duration_) {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Accumulating point clouds... (%zu pts, %.1f/%.1fs)",
        accumulated_cloud_->size(), elapsed, accumulate_duration_);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Startup accumulation complete. Running KISS-Matcher...");
    if (performRegistration()) {
      startup_reloc_done_ = true;
      RCLCPP_INFO(this->get_logger(), "Startup relocalization succeeded.");
    } else {
      // Keep accumulating and retry on next timer tick
      RCLCPP_WARN(this->get_logger(), "Startup relocalization failed, will retry.");
    }
    return;
  }

  // Runtime recovery: GICP lost tracking
  if (is_relocating_) {
    return;  // Already running
  }

  if (current_failures_ < failure_threshold_) {
    return;  // GICP healthy
  }

  const double since_last = (now - last_reloc_time_).seconds();
  if (since_last < cooldown_duration_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "GICP failures=%d (threshold=%d), but cooldown active (%.1f/%.1fs).",
      current_failures_, failure_threshold_, since_last, cooldown_duration_);
    return;
  }

  RCLCPP_WARN(
    this->get_logger(),
    "GICP lost tracking (failures=%d >= %d). Triggering KISS-Matcher recovery...",
    current_failures_, failure_threshold_);

  is_relocating_ = true;
  if (performRegistration()) {
    RCLCPP_INFO(this->get_logger(), "Recovery relocalization succeeded.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Recovery relocalization failed.");
  }
  is_relocating_ = false;
  last_reloc_time_ = this->now();
}

bool KissMatcherRelocalizationNode::performRegistration()
{
  const auto t0 = std::chrono::steady_clock::now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
      RCLCPP_WARN(this->get_logger(), "Accumulated cloud is empty, skipping registration.");
      return false;
    }
    source_cloud = accumulated_cloud_;
    accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  // Remove NaN
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices);

  // 【新增】预设位姿变换: 把 source 点云变换到 preset 附近的 map 系坐标
  Eigen::Isometry3d T_preset = Eigen::Isometry3d::Identity();
  bool has_preset = (std::abs(preset_x_) > 0.01 || std::abs(preset_y_) > 0.01 ||
                     std::abs(preset_yaw_) > 0.01);
  if (has_preset) {
    T_preset.translation() = Eigen::Vector3d(preset_x_, preset_y_, preset_z_);
    T_preset.linear() = Eigen::AngleAxisd(preset_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, T_preset.matrix().cast<float>());
    source_cloud = transformed_cloud;
    RCLCPP_INFO(this->get_logger(),
      "KISS preset transform applied: [%.2f, %.2f, %.2f, yaw=%.2f°]",
      preset_x_, preset_y_, preset_z_, preset_yaw_ * 180.0 / M_PI);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "KISS-Matcher registration: source=%zu pts, target=%zu pts, voxel=%.2f m",
    source_cloud->size(), target_cloud_->size(), voxel_resolution_);

  // Convert to vectors for KISS-Matcher
  const auto src_vec = cloudToVec(source_cloud);
  const auto tgt_vec = cloudToVec(target_cloud_);

  if (src_vec.empty() || tgt_vec.empty()) {
    RCLCPP_WARN(this->get_logger(), "Source or target vector is empty after conversion.");
    return false;
  }

  // Run KISS-Matcher
  // 【修正】fpfh_radius 默认 = 5.0 * voxel_size = 1.5m，对大场景不够。
  // 增大 fpfh_radius 到 8m，同时按比例增大 normal_radius。
  float fpfh_r_gain = 8.0f / voxel_resolution_;
  float normal_r_gain = 4.0f / voxel_resolution_;
  kiss_matcher::KISSMatcherConfig config(
    voxel_resolution_, normal_r_gain, fpfh_r_gain);
  config.use_quatro_ = use_quatro_;
  config.use_ratio_test_ = false;  // Speed up for scan-level matching

  kiss_matcher::KISSMatcher matcher(config);
  kiss_matcher::RegistrationSolution solution;
  solution = matcher.estimate(src_vec, tgt_vec);

  const size_t num_final_inliers = matcher.getNumFinalInliers();

  const auto t1 = std::chrono::steady_clock::now();
  const double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  RCLCPP_INFO(
    this->get_logger(),
    "KISS-Matcher result: inliers=%zu/%d, time=%.1f ms",
    num_final_inliers, num_inliers_threshold_, dt_ms);

  // Check quality
  if (num_final_inliers < static_cast<size_t>(num_inliers_threshold_)) {
    RCLCPP_WARN(
      this->get_logger(),
      "\033[33m=> Registration might have failed (inliers=%zu < %d)\033[0m",
      num_final_inliers, num_inliers_threshold_);
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "\033[32m=> Registration succeeded (inliers=%zu >= %d)\033[0m",
    num_final_inliers, num_inliers_threshold_);

  // Build T_map_base from solution
  // 【修正】KISS-Matcher 在对称场景下 yaw 容易产生 180° 歧义，
  // 只信任 translation，rotation 设为单位矩阵，让 GICP 自己收敛 yaw。
  Eigen::Vector3d translation = solution.translation.cast<double>();

  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
  T_map_base.translation() = translation;

  // 【新增】如果使用了 preset，结果需要与 preset 组合: T_map_base = T_kiss * T_preset
  if (has_preset) {
    T_map_base = T_map_base * T_preset;
    RCLCPP_INFO(this->get_logger(),
      "KISS result combined with preset: final_t=[%.2f, %.2f, %.2f]",
      T_map_base.translation().x(), T_map_base.translation().y(), T_map_base.translation().z());
  }

  publishInitialPose(T_map_base);
  return true;
}

void KissMatcherRelocalizationNode::publishInitialPose(const Eigen::Isometry3d & T_map_base)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = map_frame_;

  const Eigen::Vector3d t = T_map_base.translation();
  const Eigen::Quaterniond q(T_map_base.rotation());

  pose_msg.pose.pose.position.x = t.x();
  pose_msg.pose.pose.position.y = t.y();
  pose_msg.pose.pose.position.z = t.z();
  pose_msg.pose.pose.orientation.x = q.x();
  pose_msg.pose.pose.orientation.y = q.y();
  pose_msg.pose.pose.orientation.z = q.z();
  pose_msg.pose.pose.orientation.w = q.w();

  // Covariance: XY/Yaw moderate, Z/Roll/Pitch large (LIO more reliable)
  pose_msg.pose.covariance.fill(0.0);
  pose_msg.pose.covariance[0]  = 0.5;   // x
  pose_msg.pose.covariance[7]  = 0.5;   // y
  pose_msg.pose.covariance[14] = 10.0;  // z (don't trust)
  pose_msg.pose.covariance[21] = 10.0;  // roll (don't trust)
  pose_msg.pose.covariance[28] = 10.0;  // pitch (don't trust)
  pose_msg.pose.covariance[35] = 0.5;   // yaw

  initial_pose_pub_->publish(pose_msg);

  // Publish status
  std_msgs::msg::Int32 status;
  status.data = 1;  // success
  status_pub_->publish(status);

  RCLCPP_INFO(
    this->get_logger(),
    "Published /initialpose: t=[%.3f, %.3f, %.3f] yaw=%.2f deg",
    t.x(), t.y(), t.z(),
    std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())) * 180.0 / M_PI);
}

std::vector<Eigen::Vector3f> KissMatcherRelocalizationNode::cloudToVec(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
{
  std::vector<Eigen::Vector3f> vec;
  vec.reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    vec.emplace_back(pt.x, pt.y, pt.z);
  }
  return vec;
}

}  // namespace kiss_matcher_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiss_matcher_relocalization::KissMatcherRelocalizationNode)
