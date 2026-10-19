#ifndef SERIAL_SEND_H
#define SERIAL_SEND_H

// C++ 标准库
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

// ROS 2 相关库
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

// 串口库
#include <serial/serial.h>

// ==========================================
// 协议常量
// ==========================================
// TX 发送帧常量 (NUC → MCU)
constexpr uint8_t FRAME_HEAD      = 0xAA;
constexpr uint8_t FRAME_TAIL      = 0xDD;
constexpr uint8_t FRAME_TYPE_TX   = 0x01;  // NUC → MCU
constexpr size_t  FRAME_LEN       = 11;    // 1(头) + 1(类型) + 1(arrive) + 6(vx/vy/wz) + 1(stance 1-6) + 1(尾)

// RX 接收帧常量 (MCU → NUC)
constexpr uint8_t RX_FRAME_HEAD   = 0xBB;
constexpr uint8_t RX_FRAME_TAIL   = 0xFF;
constexpr size_t  RX_FRAME_LEN    = 10;    // 1(头) + 8(数据) + 1(尾校验0xFF), [7] defense(0/1/2), [8] outpost_hp

// ==========================================
// 1. TX 发送帧 (NUC → MCU, 11 字节)
// ==========================================
#pragma pack(push, 1)
union Serial_Package
{
    uint8_t buffer[FRAME_LEN];

    struct
    {
        uint8_t head;       // [0] 帧头 0xAA
        uint8_t frame_type; // [1] 帧类型 0x01
        uint8_t arrive;     // [2] 到达标志位 (0:导航中, 1:已到达)
        uint8_t vx_high;    // [3] X轴速度 高字节
        uint8_t vx_low;     // [4] X轴速度 低字节
        uint8_t vy_high;    // [5] Y轴速度 高字节
        uint8_t vy_low;     // [6] Y轴速度 低字节
        uint8_t theta_high; // [7] 角速度 高字节
        uint8_t theta_low;  // [8] 角速度 低字节
        uint8_t stance;     // [9] 姿态指令 (1:进攻,2:防御,3:移动, 4:强化进攻,5:强化防御,6:强化移动)
        uint8_t tail;       // [10] 帧尾 0xDD
    } data;

    Serial_Package() {
        std::memset(buffer, 0, sizeof(buffer));
        data.head = FRAME_HEAD;
        data.frame_type = FRAME_TYPE_TX;
        data.arrive = 0;
        data.stance = 3;
        data.tail = FRAME_TAIL;
    }


    // 浮点数压缩: float → 2 字节 (原有算法, 未修改)
    void setCompressedFloat(float val, uint8_t& high_ref, uint8_t& low_ref) {
        double integer_part;
        double fraction_part = std::modf(static_cast<double>(val), &integer_part);

        double int_abs = (std::abs(integer_part) > 63.0) ? 63.0 : std::abs(integer_part);
        uint8_t int_val = static_cast<uint8_t>(int_abs);

        if (val < 0) {
            high_ref = 0x40 + int_val;
        } else {
            high_ref = int_val;
        }

        double frac_abs = std::round(std::abs(fraction_part * 100.0));
        low_ref = static_cast<uint8_t>(frac_abs);
    }
};
#pragma pack(pop)

// ==========================================
// 2. RX 接收帧 (MCU → NUC, 10 字节)
// ==========================================
#pragma pack(push, 1)
struct Rx_Frame
{
    uint8_t head;            // [0]  帧头 0xBB
    uint8_t game_time_low;   // [1]  比赛时间 低字节
    uint8_t game_time_high;  // [2]  比赛时间 高字节
    uint8_t hp_low;          // [3]  机器人血量 低字节
    uint8_t hp_high;         // [4]  机器人血量 高字节
    uint8_t protect;         // [5]  保护标志 (1=去高地, 0=解除)
    uint8_t ammo;            // [6]  低弹量告警 (1=弹量<20 需补弹, 0=弹量≥20 充足)
    uint8_t defense;         // [7]  防守标志位 (0=无, 1=保护泉水, 2=保护英雄)
    uint8_t outpost_hp;      // [8]  前哨站存活状态 (1=存活, 0=死亡)
    uint8_t tail;            // [9]  帧尾 0xFF
};
#pragma pack(pop)

static_assert(sizeof(Serial_Package) == FRAME_LEN, "TX frame must be 11 bytes");
static_assert(sizeof(Rx_Frame)       == RX_FRAME_LEN, "RX frame must be 10 bytes");

// ==========================================
// 3. 辅助函数
// ==========================================

// 高低字节拼接为 uint16_t
inline uint16_t mergeU16(uint8_t high, uint8_t low)
{
    return (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
}

// ==========================================
// 4. ROS 2 节点类定义 (TX + RX)
// ==========================================
class Serial_Node : public rclcpp::Node
{
public:
    Serial_Node(const std::string& cli_port = "", float cli_scale = 1.0f)
    : Node("Serial_Node")
    {
        // ---- 参数声明 ----
        std::string cmd_vel_topic = this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        std::string default_port = cli_port.empty() ? "/dev/ttyUSB1" : cli_port;
        std::string port_name = this->declare_parameter<std::string>("port_name", default_port);
        speed_scale_ = this->declare_parameter<float>("speed_scale", cli_scale);
        int baud_rate = this->declare_parameter<int>("baud_rate", 500000);
        
        // arrive 默认值参数 (0=导航中, 1=已到达)
        int default_arrive = this->declare_parameter<int>("default_arrive", 0);
        current_arrive_.store(static_cast<uint8_t>(default_arrive));
        RCLCPP_INFO(this->get_logger(), "Default arrive: %d", default_arrive);

        RCLCPP_INFO(this->get_logger(), "Init Config -> Port: %s, Speed Scale: %.2f",
            port_name.c_str(), speed_scale_);

        // ---- 串口初始化 ----
        try {
            serial_port_.setPort(port_name);
            serial_port_.setBaudrate(baud_rate);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
        }

        if (serial_port_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial Port Initialized Successfully.");
        }

        // ---- TX: 订阅 /cmd_vel (缓存最新值) ----
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
                latest_cmd_vel_ = *msg;
            });

        // ---- TX: 订阅 /arrive (行为树发布到达标志位) ----
        auto arrive_qos = rclcpp::QoS(1).reliable().transient_local();
        arrive_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/arrive", arrive_qos,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                current_arrive_.store(msg->data);
                RCLCPP_INFO(this->get_logger(), "[TX] arrive updated to %u", msg->data);
            });

        // ---- TX: 订阅 /stance (行为树发布姿态指令: 1=进攻, 2=防御, 3=移动) ----
        stance_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/stance", arrive_qos,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                current_stance_.store(msg->data);
                RCLCPP_INFO(this->get_logger(), "[TX] stance updated to %u", msg->data);
            });

        // ---- TX: 启动固定频率发送线程 (与 Nav2 controller_frequency 同步，默认 30Hz) ----
        double tx_frequency = this->declare_parameter<double>("tx_frequency", 30.0);
        tx_period_ms_ = static_cast<int>(1000.0 / tx_frequency);
        tx_running_.store(true);
        tx_thread_ = std::thread(&Serial_Node::txThreadFunc, this);
        RCLCPP_INFO(this->get_logger(), "TX send thread started at %.1f Hz (period: %d ms)",
                    tx_frequency, tx_period_ms_);

        // ---- RX: 创建发布者 ----
        time_pub_    = this->create_publisher<std_msgs::msg::Float64>("/referee/game_time", 10);
        hp_pub_      = this->create_publisher<std_msgs::msg::Float64>("/referee/hp", 10);
        protect_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/referee/protect", 10);
        ammo_pub_    = this->create_publisher<std_msgs::msg::Float64>("/referee/ammo", 10);
        defense_pub_    = this->create_publisher<std_msgs::msg::UInt8>("/referee/defense", 10);
        outpost_hp_pub_ = this->create_publisher<std_msgs::msg::Float64>("/referee/outpost_hp", 10);

        // ---- RX: 启动接收线程 ----
        rx_running_.store(true);
        rx_thread_ = std::thread(&Serial_Node::rxThreadFunc, this);

        RCLCPP_INFO(this->get_logger(), "RX receive thread started.");

        // ---- 测试模式: 从终端输入 HP 值 ----
        test_mode_ = this->declare_parameter<bool>("test_mode", false);
        if (test_mode_) {
            // 默认发布 HP=400 (满血)
            std_msgs::msg::Float64 hp_msg;
            hp_msg.data = 400.0;
            hp_pub_->publish(hp_msg);

            test_thread_ = std::thread(&Serial_Node::testInputFunc, this);
            RCLCPP_WARN(this->get_logger(),
                "[TEST] 测试模式已开启, 已发送 HP=400");
            RCLCPP_WARN(this->get_logger(),
                "[TEST] 在终端输入 HP 值 (0-400) 按回车发送, 输入 q 退出");
        }
    }

    ~Serial_Node()
    {
        // 通知线程退出
        tx_running_.store(false);
        rx_running_.store(false);
        
        if (tx_thread_.joinable()) {
            tx_thread_.join();
        }
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (test_thread_.joinable()) {
            test_thread_.join();
        }

        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
    }

private:
    serial::Serial serial_port_;
    float speed_scale_;

    // ========================= TX =========================
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr arrive_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr stance_sub_;
    std::atomic<uint8_t> current_arrive_{0};  // 到达标志位（0=导航中, 1=已到达）
    std::atomic<uint8_t> current_stance_{3};  // 姿态指令 (1-6)
    
    // 缓存最新 cmd_vel
    geometry_msgs::msg::Twist latest_cmd_vel_;
    std::mutex cmd_vel_mutex_;
    
    // 固定频率发送线程
    std::thread tx_thread_;
    std::atomic<bool> tx_running_{false};
    int tx_period_ms_{33};  // 默认 30Hz = 33ms

    /**
     * @brief TX 固定频率发送线程
     * 
     * 以固定频率发送数据帧，与 Nav2 controller_frequency 同步
     * 默认 30Hz，可通过参数 tx_frequency 配置
     */
    void txThreadFunc()
    {
        RCLCPP_INFO(this->get_logger(), "[TX] Thread started, period: %d ms", tx_period_ms_);
        
        while (tx_running_.load()) {
            if (!serial_port_.isOpen()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // 获取最新 cmd_vel
            geometry_msgs::msg::Twist cmd_vel;
            {
                std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
                cmd_vel = latest_cmd_vel_;
            }

            // 打包并发送
            sendSerialPackage(cmd_vel);

            // 固定周期休眠
            std::this_thread::sleep_for(std::chrono::milliseconds(tx_period_ms_));
        }
        
        RCLCPP_INFO(this->get_logger(), "[TX] Thread stopped.");
    }

    /**
     * @brief 打包并发送串口数据帧
     */
    void sendSerialPackage(const geometry_msgs::msg::Twist& cmd_vel)
    {
        Serial_Package pkg;

        float vx_scaled = cmd_vel.linear.x * speed_scale_;
        float vy_scaled = cmd_vel.linear.y * speed_scale_;
        // wz 从 rad/s 转换为 °/s（×57.2958）后再乘 speed_scale，便于 MCU 直接以度为单位控制。
        // 注意：协议整数部分上限 63，所以 Nav2 max_vel_theta 必须 ≤ 1.0 rad/s ≈57°/s。
        float wz_scaled = cmd_vel.angular.z * 57.2958f * speed_scale_;

        pkg.setCompressedFloat(vx_scaled, pkg.data.vx_high, pkg.data.vx_low);
        pkg.setCompressedFloat(vy_scaled, pkg.data.vy_high, pkg.data.vy_low);
        pkg.setCompressedFloat(wz_scaled, pkg.data.theta_high, pkg.data.theta_low);

        pkg.data.arrive = current_arrive_.load();
        pkg.data.stance = current_stance_.load();

        try {
            serial_port_.write(pkg.buffer, sizeof(pkg.buffer));

            // 【节流】TX 30Hz 如果每帧都打会刷屏, 节流为每秒一行 (仅验证 TX 在工作)
            // 需要逐帧详情请启动时加 --log-level debug
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[TX] Send [Scale:%.1f] -> Vx[%.2f m/s], Vy[%.2f m/s], Wz[%.2f °/s], Arrive[%u], Stance[%u]",
                speed_scale_, vx_scaled, vy_scaled, wz_scaled, pkg.data.arrive, pkg.data.stance);
        } catch (serial::IOException& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[TX] Serial write failed: %s", e.what());
        }
    }

    // ========================= RX =========================
    std::thread rx_thread_;
    std::atomic<bool> rx_running_{false};

    // ROS 2 发布者
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hp_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr   protect_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ammo_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr   defense_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr outpost_hp_pub_;

    /**
     * @brief RX 接收线程主函数
     *
     * 解包流程 (在独立线程中循环执行):
     *   1. 逐字节扫描, 寻找帧头 0xBB
     *   2. 找到帧头后, 读取剩余 9 字节凑满 10 字节帧
     *   3. 逐字段解析:
     *      - [1-2] game_time:     小端拼接为 uint16
     *      - [3-4] hp:            小端拼接为 uint16
     *      - [5]   protect:       保护标志 (1=去高地, 0=解除)
     *      - [6]   ammo:          低弹量告警 (1=<20 需补弹, 0=≥20 充足)
     *      - [7]   defense:       防守标志位 (0=无, 1=保护泉水, 2=保护英雄)
     *      - [8]   outpost_hp:    前哨站存活状态 (1=存活, 0=死亡)
     *      - [9]   tail:          帧尾 0xFF
     *   5. 发布至 ROS 2 话题
     */
    void rxThreadFunc()
    {
        uint8_t buf[RX_FRAME_LEN + 1];

        while (rx_running_.load())
        {
            // 串口未就绪时等待
            if (!serial_port_.isOpen()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            try {
                // ---- 步骤 1: 逐字节扫描, 寻找帧头 0xBB ----
                if (serial_port_.available() == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                uint8_t byte;
                serial_port_.read(&byte, 1);
                if (byte != RX_FRAME_HEAD) {
                    continue;  // 不是帧头, 丢弃继续扫描
                }
                buf[0] = byte;

                // ---- 步骤 2: 读取剩余 9 字节 ----
                size_t remaining = RX_FRAME_LEN - 1;
                size_t read_cnt = serial_port_.read(buf + 1, remaining);
                if (read_cnt != remaining) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "[RX] Incomplete frame: got %zu / %zu bytes", read_cnt, remaining);
                    continue;
                }

                // ---- 步骤 3: 校验帧尾 0xFF ----
                if (buf[RX_FRAME_LEN - 1] != RX_FRAME_TAIL) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "[RX] Tail mismatch: expected 0x%02X, got 0x%02X, skip",
                        RX_FRAME_TAIL, buf[RX_FRAME_LEN - 1]);
                    continue;
                }

                // ---- 步骤 4: 解析各字段 ----
                const auto* rx = reinterpret_cast<const Rx_Frame*>(buf);

                uint16_t game_time  = mergeU16(rx->game_time_high, rx->game_time_low); // [1-2] 小端
                uint16_t hp         = mergeU16(rx->hp_high, rx->hp_low);                // [3-4] 小端
                uint8_t  protect_v  = rx->protect;                                     // [5]
                uint8_t  ammo       = rx->ammo;                                        // [6]
                uint8_t  defense_v  = rx->defense;                                     // [7]
                uint8_t  outpost_hp  = rx->outpost_hp;                                 // [8]

                // ---- 步骤 5: 数据有效性检查 ----
                // HP 范围: 0-400 (RoboMaster 哨兵最大血量)
                if (hp > 400) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[RX] 数据异常: hp=%u，丢弃本帧", hp);
                    continue;
                }
                // 全字段范围校验, 拒绝因数据内 0xBB 导致的错位帧
                if (game_time > 420 || protect_v > 1 || ammo > 1 || defense_v > 2 || outpost_hp > 1) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "[RX] 错位帧: time=%u hp=%u protect=%u ammo=%u defense=%u outpost_hp=%u",
                        game_time, hp, protect_v, ammo, defense_v, outpost_hp);
                    continue;
                }

                // ---- 步骤 6: 发布至 ROS 2 话题 ----
                std_msgs::msg::Float64 time_msg;
                time_msg.data = static_cast<double>(game_time);
                time_pub_->publish(time_msg);

                // 测试模式下 HP 由终端输入线程发布, 跳过串口 HP
                if (!test_mode_) {
                    std_msgs::msg::Float64 hp_msg;
                    hp_msg.data = static_cast<double>(hp);
                    hp_pub_->publish(hp_msg);
                }

                std_msgs::msg::UInt8 protect_msg;
                protect_msg.data = protect_v;
                protect_pub_->publish(protect_msg);

                std_msgs::msg::Float64 ammo_msg;
                ammo_msg.data = static_cast<double>(ammo);  // 0.0 或 1.0，与行为树 Update/CheckAmmo 以 Float64 形式兑接
                ammo_pub_->publish(ammo_msg);

                std_msgs::msg::UInt8 defense_msg;
                defense_msg.data = defense_v;
                defense_pub_->publish(defense_msg);

                std_msgs::msg::Float64 outpost_hp_msg;
                outpost_hp_msg.data = static_cast<double>(outpost_hp);
                outpost_hp_pub_->publish(outpost_hp_msg);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "[RX] time=%u hp=%u protect=%u ammo=%u defense=%u outpost_hp=%u",
                    game_time, hp, protect_v, ammo, defense_v, outpost_hp);
                // 原始字节 (每次有效帧打印, 调试用)
                RCLCPP_DEBUG(this->get_logger(),
                    "[RX] raw: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);

            } catch (serial::IOException& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[RX] Serial read error: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        RCLCPP_INFO(this->get_logger(), "[RX] Receive thread stopped.");
    }

    // ========================= TEST =========================
    bool test_mode_{false};
    std::thread test_thread_;

    /**
     * @brief 测试模式终端输入线程
     *
     * 从 stdin 读取 HP 值并发布到 /referee/hp, 用于调试行为树撤退逻辑.
     * 输入 q 或 Ctrl+D 退出.
     */
    void testInputFunc()
    {
        std::string line;
        while (rx_running_.load()) {
            std::cout << "[TEST] 输入 HP 值 > " << std::flush;
            if (!std::getline(std::cin, line)) {
                break;  // EOF (Ctrl+D)
            }
            if (line == "q" || line == "Q" || line == "quit") {
                RCLCPP_INFO(this->get_logger(), "[TEST] 退出测试输入");
                break;
            }
            try {
                double hp_val = std::stod(line);
                std_msgs::msg::Float64 hp_msg;
                hp_msg.data = hp_val;
                hp_pub_->publish(hp_msg);
                RCLCPP_INFO(this->get_logger(), "[TEST] 已发布 HP=%.0f", hp_val);
            } catch (const std::exception&) {
                std::cout << "[TEST] 无效输入, 请输入数字 (0-400) 或 q 退出" << std::endl;
            }
        }
    }
};

#endif // SERIAL_SEND_H
