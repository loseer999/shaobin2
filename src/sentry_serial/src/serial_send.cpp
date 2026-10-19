#include "serial_send.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <string>

int main(int argc, char ** argv)
{
    // 1. 初始化 ROS 2 上下文
    rclcpp::init(argc, argv);

    // 2. 解析命令行参数
    std::string port_name = "";
    float speed_scale = 1.0f;

    // 获取去除 ROS 系统特定参数（如 --ros-args 等）后的干净参数列表
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

    // args[0] 通常是程序执行路径，所以真正的参数从 args[1] 开始
    if (args.size() > 1) {
        port_name = args[1]; // 第一个参数: 串口名称
    }
    
    if (args.size() > 2) {
        try {
            speed_scale = std::stof(args[2]); // 第二个参数: 速度系数，转为 float
        } catch (const std::exception& e) {
            std::cout << "[Warning] Invalid speed scale input, defaulting to 1.0" << std::endl;
            speed_scale = 1.0f;
        }
    }

    // 3. 创建节点并将参数传入
    auto node = std::make_shared<Serial_Node>(port_name, speed_scale);

    // 4. 开始自旋 (Spin)
    rclcpp::spin(node);

    // 5. 关闭 ROS 2 上下文
    rclcpp::shutdown();

    return 0;
}
