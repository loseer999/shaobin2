// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <chrono>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  // 【业务收敛半径】默认 0.2 m → 单帧 GICP 输出位移超过此值视为跳变 (不算收敛)。
  this->declare_parameter("convergence_radius", 0.5);  // 【调优】0.5m 平衡初值容忍与跳变防护
  // 【失败容忍】连续未收敛帧数上限。默认 100 帧 → 根据 register_timer_ 频率计
  // 算 (当前 0.5 Hz → 约 200s)。 <= 此值时仅 WARN, 超过让业务代码看到 ERROR。
  this->declare_parameter("max_consecutive_failures", 100);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("convergence_radius", convergence_radius_);
  this->get_parameter("max_consecutive_failures", max_consecutive_failures_);
  consecutive_failures_ = 0;
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", 10);
  failures_pub_ = this->create_publisher<std_msgs::msg::Int32>("/gicp/consecutive_failures", 10);

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(2000),  // 0.5 Hz: 让 source 累积 ~20 帧 避免协方差退化 (原 500ms → 单帧只 ~370 点 太稀疏)
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points in map frame", global_map_->points.size());
  // PCD 由 Point-LIO 在 map/odom 坐标系下保存，直接作为 target，无需 TF 变换
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  // 校验 frame_id 必须是 base_link（/cloud_registered_body）
  if (msg->header.frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Expected frame_id=%s, got %s. Skipping.", base_frame_.c_str(),
      msg->header.frame_id.c_str());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);

  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  static int reg_count = 0;
  ++reg_count;

  const auto t0 = std::chrono::steady_clock::now();
  const size_t raw_pts = accumulated_cloud_->size();

  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(),
      "\033[33m[GICP #%d] \u26a0\ufe0f  source \u7d2f\u79ef\u70b9\u4e3a\u7a7a, \u8df3\u8fc7\033[0m", reg_count);
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 200;  // 【增大】原 50 不够, 需更多迭代避免局部最小值
  register_->criteria.translation_eps = 5e-3;  // 【放宽】 5mm 平移阈值, 更容易收敛
  register_->criteria.rotation_eps    = 5e-3;  // 【放宽】 ~0.29° 旋转阈值, 更容易收敛

  // 【详细日志 1/3】入口信息: 累积点数 + 体素下采样后点数 + GICP 起始位姿
  const Eigen::Vector3d t_prev = previous_result_t_.translation();
  const Eigen::Vector3d r_prev = previous_result_t_.rotation().eulerAngles(0, 1, 2) * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(),
    "\n\033[36m╔══ GICP #%d ══════════════════════════\033[0m\n"
    "\033[36m║ source: raw=%zu → voxel(%g m)=%zu pts\033[0m\n"
    "\033[36m║ target: %zu pts | match_dist=%.2f m | conv_radius=%.2f m | iter≤%d\033[0m\n"
    "\033[36m║ 初值 (上次结果): t=[%.3f, %.3f, %.3f] rpy=[%.2f, %.2f, %.2f]°\033[0m",
    reg_count, raw_pts, registered_leaf_size_, source_->size(),
    target_->size(), std::sqrt(max_dist_sq_), convergence_radius_, register_->optimizer.max_iterations,
    t_prev.x(), t_prev.y(), t_prev.z(), r_prev.x(), r_prev.y(), r_prev.z());

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  const auto t1 = std::chrono::steady_clock::now();
  const double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 【详细日志 2/3】出口: 收敛状态 + 迭代次数 + inlier + error + 耗时
  const Eigen::Vector3d t_new = result.T_target_source.translation();
  const Eigen::Vector3d r_new = result.T_target_source.rotation().eulerAngles(0, 1, 2) * 180.0 / M_PI;
  const Eigen::Vector3d dt = t_new - t_prev;
  const double dt_norm = dt.norm();

  // 【修正】检测 GICP 是否收敛到翻转位姿（对称场景常见问题）
  const Eigen::Matrix3d R = result.T_target_source.rotation();
  const double det = R.determinant();
  const bool is_flipped = (det < 0.0) || (R(2, 2) < 0.0);

  if (result.converged && !is_flipped) {
    if (dt_norm <= convergence_radius_) {
      // ✅ 业务收敛: GICP 数值收敛 + 位移在阈值内 + 无翻转
      result_t_ = previous_result_t_ = result.T_target_source;
      consecutive_failures_ = 0;  // 【失败容忍】业务收敛即清零

      // 【新增】发布 map→base_link 的绝对位姿给 EKF
      {
        // result_t_ 直接是 map→base_link
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = last_scan_time_;
        pose_msg.header.frame_id = map_frame_;
        const Eigen::Vector3d t = result_t_.translation();
        const Eigen::Quaterniond q(result_t_.rotation());
        pose_msg.pose.pose.position.x = t.x();
        pose_msg.pose.pose.position.y = t.y();
        pose_msg.pose.pose.position.z = t.z();
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        // 协方差: XY 给小值 (GICP 精度高)，Z/Roll/Pitch 给大值 (LIO 更可靠)
        pose_msg.pose.covariance[0]  = 0.01;  // x
        pose_msg.pose.covariance[7]  = 0.01;  // y
        pose_msg.pose.covariance[14] = 1.0;   // z (EKF 不信)
        pose_msg.pose.covariance[21] = 1.0;   // roll (EKF 不信)
        pose_msg.pose.covariance[28] = 1.0;   // pitch (EKF 不信)
        pose_msg.pose.covariance[35] = 0.02;  // yaw
        pose_pub_->publish(pose_msg);
      }
      RCLCPP_INFO(this->get_logger(),
        "\033[32m║ ✅ 收敛: iter=%zu/%d  inlier=%zu  err=%.4f  耗时=%.1fms\033[0m\n"
        "\033[32m║ 结果: t=[%.3f, %.3f, %.3f] rpy=[%.2f, %.2f, %.2f]°  Δt=%.4fm/%.2fm\033[0m\n"
        "\033[36m╚════════════════════════════════\033[0m",
        result.iterations, register_->optimizer.max_iterations,
        result.num_inliers, result.error, dt_ms,
        t_new.x(), t_new.y(), t_new.z(), r_new.x(), r_new.y(), r_new.z(),
        dt_norm, convergence_radius_);
    } else {
      // ⚠️ GICP 数值收敛但位移跳超阈 → 拒绝更新 previous, 保护上次稳定结果
      ++consecutive_failures_;
      if (consecutive_failures_ <= max_consecutive_failures_) {
        RCLCPP_WARN(this->get_logger(),
          "\033[33m║ ⚠️  位姿跳变拒绝: Δt=%.4fm > %.2fm (阈值)\033[0m\n"
          "\033[33m║ 被GICP变换到: t=[%.3f, %.3f, %.3f] rpy=[%.2f, %.2f, %.2f]°  iter=%zu  inlier=%zu\033[0m\n"
          "\033[33m║ 连续失败 %d/%d 帧 (未超阈仅告警, 位姿保持上次)\033[0m\n"
          "\033[36m╚════════════════════════════════\033[0m",
          dt_norm, convergence_radius_,
          t_new.x(), t_new.y(), t_new.z(), r_new.x(), r_new.y(), r_new.z(),
          result.iterations, result.num_inliers,
          consecutive_failures_, max_consecutive_failures_);
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "\033[1;31m║ 🔥 连续位姿跳变 %d/%d 帧, 已超阈! GICP 可能已脱锁\033[0m\n"
          "\033[1;31m║ Δt=%.4fm > %.2fm (阈值)  iter=%zu  inlier=%zu  err=%.4f\033[0m\n"
          "\033[1;31m║ 被GICP变换到: t=[%.3f, %.3f, %.3f] rpy=[%.2f, %.2f, %.2f]°\033[0m\n"
          "\033[1;31m║ 建议: 检查初值、点云质量, 或 RViz 发 /initialpose 重置\033[0m\n"
          "\033[36m╚════════════════════════════════\033[0m",
          consecutive_failures_, max_consecutive_failures_,
          dt_norm, convergence_radius_,
          result.iterations, result.num_inliers, result.error,
          t_new.x(), t_new.y(), t_new.z(), r_new.x(), r_new.y(), r_new.z());
      }
    }
  } else {
    // ❌ GICP 数值未收敛
    ++consecutive_failures_;
    if (consecutive_failures_ <= max_consecutive_failures_) {
      RCLCPP_WARN(this->get_logger(),
        "\033[31m║ ❌ 未收敛: iter=%zu/%d  inlier=%zu  err=%.4f  耗时=%.1fms\033[0m\n"
        "\033[31m║ 末态: t=[%.3f, %.3f, %.3f] rpy=[%.2f, %.2f, %.2f]°  Δt=%.4fm\033[0m\n"
        "\033[33m║ 连续失败 %d/%d 帧 (未超阈仅告警)\033[0m\n"
        "\033[36m╚════════════════════════════════\033[0m",
        result.iterations, register_->optimizer.max_iterations,
        result.num_inliers, result.error, dt_ms,
        t_new.x(), t_new.y(), t_new.z(), r_new.x(), r_new.y(), r_new.z(), dt_norm,
        consecutive_failures_, max_consecutive_failures_);
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "\033[1;31m║ 🔥 连续未收敛 %d/%d 帧, 已超阈! GICP 可能已脱锁\033[0m\n"
        "\033[1;31m║ iter=%zu/%d  inlier=%zu  err=%.4f  耗时=%.1fms\033[0m\n"
        "\033[1;31m║ 末态: t=[%.3f, %.3f, %.3f] rpy=[%.2f, %.2f, %.2f]°  Δt=%.4fm\033[0m\n"
        "\033[1;31m║ 建议: 检查初值、点云质量, 或 RViz 发 /initialpose 重置\033[0m\n"
        "\033[36m╚════════════════════════════════\033[0m",
        consecutive_failures_, max_consecutive_failures_,
        result.iterations, register_->optimizer.max_iterations,
        result.num_inliers, result.error, dt_ms,
        t_new.x(), t_new.y(), t_new.z(), r_new.x(), r_new.y(), r_new.z(), dt_norm);
    }
  }

  // 【新增】发布连续失败计数 (KISS-Matcher 监控用)
  std_msgs::msg::Int32 failures_msg;
  failures_msg.data = consecutive_failures_;
  failures_pub_->publish(failures_msg);

  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  // result_t_ 是 map→base_link，查询 base_link→odom 得到 map→odom
  try {
    auto base_to_odom = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_, tf2::TimePointZero);
    Eigen::Isometry3d T_base_odom = tf2::transformToEigen(base_to_odom.transform);
    Eigen::Isometry3d T_map_odom = result_t_ * T_base_odom;

    geometry_msgs::msg::TransformStamped transform_stamped;
    // 【修】原代码用 last_scan_time_+0.1s (未来时间) → RViz 查询 cloud 老时间戳时 TF 还未发出 → status 在 OK/Error 间来回切换。
    // 改用 now() (当前时间) 发布 → TF buffer 从启动到现在的区间都被覆盖 (transform_timer_ 20Hz)。
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = map_frame_;
    transform_stamped.child_frame_id = odom_frame_;

    const Eigen::Vector3d translation = T_map_odom.translation();
    const Eigen::Quaterniond rotation(T_map_odom.rotation());

    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();

    tf_broadcaster_->sendTransform(transform_stamped);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Cannot publish map→odom: %s", ex.what());
  }
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
  map_to_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  map_to_base.linear() = q.toRotationMatrix();

  previous_result_t_ = result_t_ = map_to_base;
  RCLCPP_INFO(this->get_logger(),
    "Initial pose set: map→base_link [%.2f, %.2f, %.2f] yaw=%.2f°",
    map_to_base.translation().x(), map_to_base.translation().y(), map_to_base.translation().z(),
    std::atan2(q.toRotationMatrix()(1,0), q.toRotationMatrix()(0,0)) * 180.0 / M_PI);
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
