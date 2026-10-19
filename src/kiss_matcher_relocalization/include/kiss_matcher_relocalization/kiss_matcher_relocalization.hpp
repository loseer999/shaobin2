#ifndef KISS_MATCHER_RELOCALIZATION__KISS_MATCHER_RELOCALIZATION_HPP_
#define KISS_MATCHER_RELOCALIZATION__KISS_MATCHER_RELOCALIZATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "kiss_matcher/KISSMatcher.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace kiss_matcher_relocalization
{

class KissMatcherRelocalizationNode : public rclcpp::Node
{
public:
  explicit KissMatcherRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void failuresCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void checkAndRelocate();
  bool performRegistration();
  void publishInitialPose(const Eigen::Isometry3d & map_to_base);
  std::vector<Eigen::Vector3f> cloudToVec(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr failures_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr check_timer_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string map_frame_;
  std::string robot_base_frame_;
  std::string prior_pcd_file_;
  std::string input_cloud_topic_;
  std::string failures_topic_;
  float voxel_resolution_;
  double accumulate_duration_;
  int failure_threshold_;
  double cooldown_duration_;
  bool use_quatro_;
  int num_inliers_threshold_;
  double preset_x_;
  double preset_y_;
  double preset_z_;
  double preset_yaw_;

  // State
  bool startup_reloc_done_;
  bool is_relocating_;
  rclcpp::Time last_reloc_time_;
  rclcpp::Time startup_time_;
  int current_failures_;

  // Data
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  std::mutex cloud_mutex_;
};

}  // namespace kiss_matcher_relocalization

#endif  // KISS_MATCHER_RELOCALIZATION__KISS_MATCHER_RELOCALIZATION_HPP_
