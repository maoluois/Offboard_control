#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include <chrono>
#include <cmath>
#include <thread>
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class PID {
public:
    PID(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0) {}

    double compute(double target_pos, double current_pos, double current_vel, double dt) {
        double error = target_pos - current_pos;
        integral_ += error * dt;            //飞机位置误差做积分项
        double derivative = -current_vel;  // 使用飞控反馈速度作为微分项
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        //限幅
        return std::clamp(output, -2.0, 2.0);
    }

    void reset() {
        integral_ = 0;
    }

private:
    double kp_, ki_, kd_;
    double integral_;
};

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl()
        : Node("offboard_control"),
          step_(0), flag_(0),
          pid_x_(0.8, 0.0, 0.2), //分别设置三轴PID
          pid_y_(0.8, 0.0, 0.2),
          pid_z_(0.8, 0.0, 0.1)
    {
        //状态接收器初始化
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                current_state_ = *msg;
            });
        //获取飞机位姿数据
       // MAVROS 的 /pose 话题订阅使用 BestEffort
         pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
             "mavros/local_position/pose",
              rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            current_pose_ = *msg;
           });

        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "mavros/local_position/velocity",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort(),
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            current_vel_ = *msg;
           });        

        //姿态发布器初始化
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "mavros/setpoint_velocity/cmd_vel", 10);
        //舵机控制初始化
        servo1_pub_ = this->create_publisher<std_msgs::msg::Int32>("servo1_cmd", 10);

        servo2_pub_ = this->create_publisher<std_msgs::msg::Int32>("servo2_cmd", 10);
 
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        timer_ = this->create_wall_timer(
            50ms, std::bind(&OffboardControl::timer_callback, this));

        last_request_time_ = this->now();

        start_time_ = this->now();

        
    }

    bool is_connected() const { return current_state_.connected; }


private:
    void timer_callback() {
        if (!current_state_.connected) return;

        double dt = 0.05;
        switch (step_) {
        case 0:
            handle_init_phase();
            break;

        case 1:
    if (flag_ == 0) {
        flag_ = fly_to_target(0.0, 0.0, 1.0, dt); //假设在位置（1.5.0.0.1.0投放物块）
    } else {
         fly_to_target(0.0, 0.0, 1.0, dt);
        if (!hold_position_start_) {
          hold_pisition_start_time_= this->now();
            hold_position_start_ = true;
            RCLCPP_INFO(this->get_logger(), "holding position");
        } else {
            // 等待 15 秒后再进入下一步
            auto elapsed = this->now() - hold_pisition_start_time_;
            if (elapsed.seconds() >= 15.0) {
                RCLCPP_INFO(this->get_logger(), "exit holding position");
                pid_x_.reset(); pid_y_.reset(); pid_z_.reset();
                step_ = 2; flag_ = 0;
                servo_action_started_ = false;  // 清除状态
            }
        }
    }
            break;
/*
        case 2:
            if (flag_ == 0) {
                flag_ = fly_to_target(1.0, 0.0, 1.0, dt);
            } else {
                RCLCPP_INFO(this->get_logger(), "Reached step 2");
                pid_x_.reset(); pid_y_.reset(); pid_z_.reset();
                step_ = 3; flag_ = 0;
            }
            break;

       case 3: 
    if (flag_ == 0) {
        flag_ = fly_to_target(1.5, 0.0, 1.0, dt); //假设在位置（1.5.0.0.1.0投放物块）
    } else {
        if (!servo_action_started_) {
            control_servo(1,90);  // 发送舵机序号(舵机1对应GPIO 18号)和角度
            servo_action_start_time_ = this->now();  // 记录时间
            servo_action_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Servo turning... waiting 1s before next step");
        } else {
            // 等待 1 秒后再进入下一步
            auto elapsed = this->now() - servo_action_start_time_;
            if (elapsed.seconds() >= 1.0) {
                RCLCPP_INFO(this->get_logger(), "Reached step 3 - servo finished");
                pid_x_.reset(); pid_y_.reset(); pid_z_.reset();
                step_ = 4; flag_ = 0;
                servo_action_started_ = false;  // 清除状态
            }
        }
    }
    break;

        case 4:
            if (flag_ == 0) {
                flag_ = fly_to_target(1.0, 0.0, 1.0, dt);
            } else {
                RCLCPP_INFO(this->get_logger(), "Reached step 4. Holding position.");
                //误差指令，准备计算下一个点的PID
                pid_x_.reset(); pid_y_.reset(); pid_z_.reset();
                step_ = 5; flag_ = 0;
            }
            break;

        case 5:
            if (flag_ == 0) {
                flag_ = fly_to_target(0.0, 0.0, 1.0, dt);
            } else {
                RCLCPP_INFO(this->get_logger(), "Reached step 5. Holding position.");
                publish_velocity(0, 0, 0);
                step_ = 6; flag_ = 0;
            }
            break;
*/
       case 2:
    if (flag_ == 0) {
        flag_ = fly_to_target(0.0, 0.0, 0.16, dt);  // 下降到指定位置
    } else {
        RCLCPP_INFO(this->get_logger(), "landing.");

        auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();

        arm_req->value = false;

         arming_client_->async_send_request(arm_req);
        // 等待 3 秒，确保 PX4 收到命令
        rclcpp::sleep_for(std::chrono::seconds(3));

        RCLCPP_INFO(this->get_logger(), "Disarm request sent. Shutting down...");

        rclcpp::shutdown();   // 关闭节点
    }
    break;

            
        }
    }

    void handle_init_phase() {
    // 发布固定微小速度使 setpoint 保持 FCU 接受 OFFBOARD 模式
      geometry_msgs::msg::TwistStamped vel;
    vel.header.stamp = this->now();
    vel.twist.linear.x = 0.0;
    vel.twist.linear.y = 0.0;
    vel.twist.linear.z = 0.1;  // 微小上升速度触发控制器激活
    vel_pub_->publish(vel); // 只发布 velocity

    // 模式未切换则切换
    if (current_state_.mode != "OFFBOARD") {
        if ((this->now() - last_request_time_).seconds() > 2.0) {
            //实际飞行需要注释掉！如果通过程序切offboard，飞机失控时遥控器将无法接管，注释掉程序会一直等待遥控器切入offboard

           // auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
           // mode_req->custom_mode = "OFFBOARD";
            //set_mode_client_->async_send_request(mode_req);

            last_request_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode...");
        }
        return;
    }

    // 解锁
    if (!current_state_.armed) {
        if ((this->now() - last_request_time_).seconds() > 1.0) {
            auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_req->value = true;
            arming_client_->async_send_request(arm_req);
            last_request_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Requesting arming...");
        }
        return;
    }

    RCLCPP_INFO(this->get_logger(), "OFFBOARD & armed, proceed to step 1");
    step_ = 1;
    flag_ = 0;
}

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


    int fly_to_target(double tx, double ty, double tz, double dt) {
        double ex = tx - current_pose_.pose.position.x;
        double ey = ty - current_pose_.pose.position.y;
        double ez = tz - current_pose_.pose.position.z;

        double vx = pid_x_.compute(tx, current_pose_.pose.position.x, current_vel_.twist.linear.x, dt);
        double vy = pid_y_.compute(ty, current_pose_.pose.position.y, current_vel_.twist.linear.y, dt);
        double vz = pid_z_.compute(tz, current_pose_.pose.position.z, current_vel_.twist.linear.z, dt);

        publish_velocity(vx, vy, vz);
        //输出速度
        RCLCPP_INFO(this->get_logger(), "vx = %.2lf vy = %.2lf vz = %.2lf",vx,vy,vz);
        
        double dist = std::sqrt(ex * ex + ey * ey + ez * ez);
        return (dist < 0.1) ? 1 : 0;
    }

    void publish_velocity(double vx, double vy, double vz) {
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        //速度限幅
        vel.twist.linear.x = std::clamp(vx, -2.0, 2.0);
        vel.twist.linear.y = std::clamp(vy, -2.0, 2.0);
        vel.twist.linear.z = std::clamp(vz, -2.0, 2.0);
        //发布速度
        vel_pub_->publish(vel);
    }

    //舵机控制函数，直接输入要转动的角度
    void control_servo(int num,int angle) {
    std_msgs::msg::Int32 msg;
    msg.data = angle;
    if(num==1)
    {
        servo1_pub_->publish(msg);
    }
    else if(num==2)
    {
        servo2_pub_->publish(msg);
    }
    RCLCPP_INFO(this->get_logger(), "Published servo angle: %d", angle);
}

    
    int step_;
    int flag_;

    rclcpp::Time last_request_time_;
    rclcpp::Time start_time_;
    //舵机控制时间延时
    rclcpp::Time servo_action_start_time_;
    //悬停时间
    rclcpp::Time hold_pisition_start_time_;

    bool servo_action_started_ = false;

    bool hold_position_start_ = false;

    bool pid_enabled_ = false;

    

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_vel_;

    PID pid_x_, pid_y_, pid_z_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr servo1_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr servo2_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();

    while (rclcpp::ok() && !node->is_connected()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(100ms);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "Waiting for FCU connection...");
    }

    RCLCPP_INFO(node->get_logger(), "FCU connected!");
    //开启程序主循环
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
