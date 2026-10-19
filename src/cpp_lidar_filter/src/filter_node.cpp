#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include "visualization_msgs/msg/marker.hpp"

class LidarFilterNode : public rclcpp::Node
{
public:
  LidarFilterNode() : Node("lidar_filter_node")
  {
    // 默认订阅 Point-LIO 输出的完全转正的局部系点云话题
    this->declare_parameter("input_topic", "cloud_registered_body");
    this->declare_parameter("output_topic", "cloud_filtered");
    
    // 裁剪参数 (CropBox) — 底盘半径33cm+余量
    this->declare_parameter("min_x", -0.35);
    this->declare_parameter("max_x", 0.35);
    this->declare_parameter("min_y", -0.35);
    this->declare_parameter("max_y", 0.35);
    this->declare_parameter("min_z", -0.15);   // 固定在 base_link 下进行裁剪，直观稳定
    this->declare_parameter("max_z", 0.55);    
    this->declare_parameter("negative", true); // true = 挖掉中间

    this->declare_parameter("max_range", 0.0);
    this->declare_parameter("leaf_size", 0.05);

    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Listening on: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());

    // 接收端: SensorDataQoS 匹配 Point-LIO 的 BestEffort 发布
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarFilterNode::cloud_callback, this, std::placeholders::_1));

    // 发布端: Best Effort，兼容 RViz 和下游订阅
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          output_topic, rclcpp::SensorDataQoS());
          
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("crop_box_marker", 10);

    timer_ = this->create_wall_timer(
     std::chrono::milliseconds(1000), std::bind(&LidarFilterNode::publish_marker, this));
  }

private:
  void publish_marker() {
    double min_x = this->get_parameter("min_x").as_double();
    double max_x = this->get_parameter("max_x").as_double();
    double min_y = this->get_parameter("min_y").as_double();
    double max_y = this->get_parameter("max_y").as_double();
    double min_z = this->get_parameter("min_z").as_double();
    double max_z = this->get_parameter("max_z").as_double();

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->now();
    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = (max_x + min_x) / 2.0;
    marker.pose.position.y = (max_y + min_y) / 2.0;
    marker.pose.position.z = (max_z + min_z) / 2.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_x - min_x;
    marker.scale.y = max_y - min_y;
    marker.scale.z = max_z - min_z;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4;

    marker_pub_->publish(marker);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*msg, *cloud_in);

    // CropBox 裁剪 (去除车身)
    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(cloud_in);

    Eigen::Vector4f min_pt, max_pt;
    min_pt << this->get_parameter("min_x").as_double(),
              this->get_parameter("min_y").as_double(),
              this->get_parameter("min_z").as_double(), 1.0;
    max_pt << this->get_parameter("max_x").as_double(),
              this->get_parameter("max_y").as_double(),
              this->get_parameter("max_z").as_double(), 1.0;

    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.setNegative(this->get_parameter("negative").as_bool());

    pcl::PCLPointCloud2::Ptr cloud_cropped(new pcl::PCLPointCloud2);
    crop.filter(*cloud_cropped);

    // 第二层 CropBox 限远
    double max_range = this->get_parameter("max_range").as_double();
    if (max_range > 0.0) {
      pcl::CropBox<pcl::PCLPointCloud2> range_crop;
      range_crop.setInputCloud(cloud_cropped);
      Eigen::Vector4f rmin, rmax;
      rmin << -static_cast<float>(max_range), -static_cast<float>(max_range), -static_cast<float>(max_range), 1.0f;
      rmax <<  static_cast<float>(max_range),  static_cast<float>(max_range),  static_cast<float>(max_range), 1.0f;
      range_crop.setMin(rmin);
      range_crop.setMax(rmax);
      range_crop.setNegative(false);
      pcl::PCLPointCloud2::Ptr cloud_ranged(new pcl::PCLPointCloud2);
      range_crop.filter(*cloud_ranged);
      cloud_cropped = cloud_ranged;
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud_cropped, output);
    
    // 核心改进：继承输入点云的 Header
    output.header = msg->header;

    pub_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilterNode>());
  rclcpp::shutdown();
  return 0;
}