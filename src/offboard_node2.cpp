#include "rclcpp/rclcpp.hpp"                  // ROS2核心库
#include "geometry_msgs/msg/pose_stamped.hpp" // 带时间戳的位置信息
#include "mavros_msgs/msg/state.hpp"          // MAVROS状态信息
#include "mavros_msgs/srv/set_mode.hpp"       // 飞行模式设置服务
#include "mavros_msgs/srv/command_bool.hpp"   // 解锁/上锁服务
#include <chrono>                             // 时间处理库
#include <thread>                             // 线程支持库

using namespace std::chrono_literals;         // 启用时间字面量（如100ms）

// 定义无人机控制节点类
class OffboardControl : public rclcpp::Node
{
public:
    // 构造函数初始化列表
    OffboardControl() : Node("offboard_control"), // 节点名称
                       step_(0),                  // 状态机初始阶段
                       last_request_time_(this->now()), // 使用节点时钟初始化
                       init_phase_(true)          // 初始化阶段标志
    {
        // 创建状态订阅器（10表示消息队列长度）
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, 
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                current_state_ = *msg; // 使用lambda更新当前状态
            });

        // 创建位置发布器（发布到mavros/setpoint_position/local）
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);

        // 初始化服务客户端
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        // 创建100ms周期的定时器（10Hz）
        timer_ = this->create_wall_timer(
            100ms, std::bind(&OffboardControl::timer_callback, this));

        init_pose(); // 初始化位置信息
    }

    // 检查飞控连接状态的公共方法
    bool is_connected() const { return current_state_.connected; }

private:
    // 初始化位置信息函数
    void init_pose()
    {
        //pose_.header.frame_id = "map";     // 坐标系设置为map
        pose_.pose.position.x = 0.0;       // 初始X坐标
        pose_.pose.position.y = 0.0;       // 初始Y坐标
        pose_.pose.position.z = 1.0;       // 初始高度1米
        //pose_.pose.orientation.w = 1.0;    // 四元数默认朝向
    }

    // 定时器回调函数（核心控制逻辑）
    void timer_callback()
    {
        // 更新并发布位置信息（必须持续发布才能保持OFFBOARD模式）
        pose_.header.stamp = this->now();
        local_pos_pub_->publish(pose_);

        // 状态机控制逻辑
        switch (step_) {
        case 0:  // 初始化阶段
            handle_init_phase();
            break;
        case 1:  // 起飞阶段
            handle_takeoff();
            break;
        case 2:  // 第一阶段移动
            handle_movement(5.0, 0.0, 1.0, 3);
            break;
        case 3:  // 第二阶段移动
            handle_movement(5.0, 5.0, 1.0, 4);
            break;
        case 4:  // 降落阶段
            handle_landing();
            break;
        }
    }

    // 初始化阶段处理函数
    void handle_init_phase()
    {
        if (!current_state_.connected) return; // 等待飞控连接

        // 首次进入时初始化时间戳
        if (start_time_.nanoseconds() == 0) {
            start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Initialization timer started");
        }

        // 计算经过时间
        auto elapsed = (this->now() - start_time_).seconds();
        if (elapsed < 5.0) {
            // 节流日志输出（每秒1次）
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "Initializing... Keep publishing setpoints");
            return;
        }

        // 尝试进入OFFBOARD模式并解锁
        if (set_mode("OFFBOARD") && arm_drone(true)) {
            RCLCPP_INFO(this->get_logger(), "Offboard enabled and armed!");
            start_time_ = this->now(); // 重置计时器
            step_ = 1; // 进入起飞阶段
        }
    }

    // 起飞阶段处理
    void handle_takeoff()
    {
        auto elapsed = (this->now() - start_time_).seconds();
        if (elapsed > 5.0) {
            RCLCPP_INFO(this->get_logger(), "Takeoff completed");
            start_time_ = this->now();
            step_ = 2; // 进入第一阶段移动
        }
    }

    // 通用移动处理函数
    void handle_movement(float x, float y, float z, int next_step)
    {
        // 设置目标位置
        pose_.pose.position.x = x;
        pose_.pose.position.y = y;
        pose_.pose.position.z = z;

        // 检查是否到达预定时间
        auto elapsed = (this->now() - start_time_).seconds();
        if (elapsed > 5.0) {
            RCLCPP_INFO(this->get_logger(), "Waypoint reached");
            start_time_ = this->now();
            step_ = next_step; // 进入下一阶段
        }
    }

    // 降落处理函数
    void handle_landing()
    {
        if (set_mode("AUTO.LAND")) {  // 切换至自动降落模式
            RCLCPP_INFO(this->get_logger(), "Landing...");
            rclcpp::sleep_for(10s);   // 等待10秒
            
            if (arm_drone(false)) {   // 尝试上锁
                RCLCPP_INFO(this->get_logger(), "Drone disarmed");
                rclcpp::shutdown();   // 关闭节点
            }
        }
    }

    // 飞行模式设置函数
    bool set_mode(const std::string& mode)
    {
        // 防止频繁请求（2秒间隔）
        if ((this->now() - last_request_time_).seconds() < 2.0) return false;

        if (current_state_.mode != mode) {
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = mode;

            // 定义服务响应回调
            using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture;
            auto response_received_callback = [this, mode](ServiceResponseFuture future) {
                auto response = future.get();
                if (response->mode_sent) {
                    RCLCPP_INFO(this->get_logger(), "%s mode enabled", mode.c_str());
                }
            };

            // 异步发送请求
            set_mode_client_->async_send_request(request, response_received_callback);
            last_request_time_ = this->now();
            return false; // 需要等待响应
        }
        return true;
    }

    // 无人机解锁/上锁函数
    bool arm_drone(bool arm)
    {
        if (current_state_.armed == arm) return true; // 状态已满足

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;

        // 定义服务响应回调
        using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture;
        auto response_received_callback = [this, arm](ServiceResponseFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Drone %s", arm ? "armed" : "disarmed");
            }
        };

        arming_client_->async_send_request(request, response_received_callback);
        last_request_time_ = this->now();
        return false; // 需要等待响应
    }

    // 成员变量说明
    int step_;                       // 状态机当前阶段
    rclcpp::Time start_time_;        // 阶段开始时间戳
    rclcpp::Time last_request_time_; // 最后服务请求时间（用于限流）
    bool init_phase_;                // 初始化阶段标志

    // ROS2组件
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;         // 状态订阅器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_; // 位置发布器
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;       // 模式服务客户端
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;      // 解锁服务客户端
    rclcpp::TimerBase::SharedPtr timer_;                       // 定时器

    // 状态存储
    mavros_msgs::msg::State current_state_;     // 当前飞控状态
    geometry_msgs::msg::PoseStamped pose_;       // 当前目标位置
};

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv); // 初始化ROS2
    auto node = std::make_shared<OffboardControl>();

    // 等待飞控连接
    while (rclcpp::ok() && !node->is_connected()) {
        rclcpp::spin_some(node);                 // 处理回调
        std::this_thread::sleep_for(100ms);      // 避免频繁发布信息和判断，导致CPU占用过高
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, 
                           "Waiting for FCU connection...");
    }

    RCLCPP_INFO(node->get_logger(), "FCU connected!");
    rclcpp::spin(node);          // 进入事件循环
    rclcpp::shutdown();          // 关闭节点
    return 0;
}