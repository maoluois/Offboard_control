#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include <chrono>
#include <cmath>
#include <thread>
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

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
          pid_z_(1.2, 0.0, 0.35)
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

        //任务订阅器
         task_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "task_topic",  
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort(),
            [this](const std_msgs::msg::String::SharedPtr msg) {
                // 获取任务编号
                task_msg_ = msg->data;
            });

         qr_data_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "qr_data",  
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort(),
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                // 获取任务编号
                qr_data_ = msg->data;
            });            
        //姿态发布器初始化
         //pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          //   "mavros/setpoint_position/local", 10);

        // vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        //     "mavros/setpoint_velocity/cmd_vel", 10);
        raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "mavros/setpoint_raw/local", 10);
        //摄像头云台角度控制
        cam_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "control_angle", 10);
        //激光笔亮灭控制
        laser_pointer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("control_laser_pointer", 10);
        //创建串口屏控制发布器
        serial_screen_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/serial_screen_command", 10);  

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        timer_ = this->create_wall_timer(
            20ms, std::bind(&OffboardControl::timer_callback, this));

        last_request_time_ = this->now();

        start_time_ = this->now();

        
    }

    bool is_connected() const { return current_state_.connected; }


private:
    void timer_callback() {
    	auto t1 = this->now();
        if (!current_state_.connected) return;

        // double dt = 0.02;
        switch (step_) {
            case 0:
                handle_init_phase();
                break;

            case 1:
            
            if (!hold_position_start_) {
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                publish_position(0.0, 0.0, 1.3);
                RCLCPP_INFO(this->get_logger(), "reach step 1");
            } 
            else 
            {
                publish_position(0.0, 0.0, 1.3); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 10.0) {
                    RCLCPP_INFO(this->get_logger(), "go to step 2");

                    if(task_msg_ == "m111m111m111") //切入任务1
                    {
                    step_ = 2;
                    }
                    else if(task_msg_ == "m222m222m222") //切入任务2
                    {
                    step_=201;    
                    }
                    
                    hold_position_start_ = false;  // 清除状态
                }
            }
            
            break;
            //---------------------------任务1----------------------    

            case 2: // A3 (当前目标点)
            if (!hold_position_start_) {
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("A3");
                RCLCPP_INFO(this->get_logger(), "start taking stock");
            } 
            else 
            {
                fly_to_target("A3"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to A2");
                    step_ = 3;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;	

            case 3: // A2
            if (!hold_position_start_) {
                //发送A3货物编号,向串口屏n2.val赋值
                serial_screen_control(2);
                //激光笔指示
                control_laser_pointer(1,0.5);
                //转动云台

                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("A2");
                RCLCPP_INFO(this->get_logger(), "reach A3");
            } 
            else 
            {
                fly_to_target("A2"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to A1");
                    step_ = 4;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;	

            case 4: // A1
            if (!hold_position_start_) {
                 //发送A2货物编号,向串口屏n1.val赋值
                serial_screen_control(1);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("A1");
                RCLCPP_INFO(this->get_logger(), "reach A2");
            } 
            else 
            {
                fly_to_target("A1"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to A5");
                    step_ = 5;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;
           
            case 5: // A4
            if (!hold_position_start_) {
                 //发送A1货物编号,向串口屏n0.val赋值
                serial_screen_control(0);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("A4");
                RCLCPP_INFO(this->get_logger(), "reach A1");
            } 
            else 
            {
                fly_to_target("A4"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to A6");
                    step_ = 6;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 6: // A5
            if (!hold_position_start_) {
                 //发送A5货物编号,向串口屏n3.val赋值
                serial_screen_control(3);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("A5");
                RCLCPP_INFO(this->get_logger(), "reach A5");
            } 
            else 
            {
                fly_to_target("A5"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to step6"); //转云台
                    step_ = 77;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 77: // A6
            if (!hold_position_start_) {
                 //发送A5货物编号,向串口屏n4.val赋值
                serial_screen_control(4);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("A6");
                RCLCPP_INFO(this->get_logger(), "reach A5");
            } 
            else 
            {
                fly_to_target("A6"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to step6"); //转云台
                    step_ = 7;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 7: // 中间点1
            if (!hold_position_start_) {
                 //发送A6货物编号,向串口屏n5.val赋值
                serial_screen_control(5);
                //激光笔指示
                control_laser_pointer(1,0.5);
                
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                publish_position(1.80, 0.20, 1.3);
                RCLCPP_INFO(this->get_logger(), "reach A6");
            } 
            else 
            {
                publish_position(1.80, 0.20, 1.3); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to step 7");
                    step_ = 8;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 8: // 中间点2
            if (!hold_position_start_) {
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                publish_position(1.80, 0.20, 1.3);
                RCLCPP_INFO(this->get_logger(), "reach step 7");
            } 
            else 
            {
                publish_position(1.80, 0.20, 1.3); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to B1");
                    step_ = 9;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 9: // B1
            if (!hold_position_start_) {
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("B1");
                RCLCPP_INFO(this->get_logger(), "reach step8");
            } 
            else 
            {
                fly_to_target("B1"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to B2");
                    step_ = 10;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 10: // B2
            if (!hold_position_start_) {
                //发送B1货物编号,向串口屏n6.val赋值
                serial_screen_control(6);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("B2");
                RCLCPP_INFO(this->get_logger(), "reach B1");
            } 
            else 
            {
                fly_to_target("B2"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to B3");
                    step_ = 11;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 11: // B3
            if (!hold_position_start_) {
                //发送B2货物编号,向串口屏n7.val赋值
                serial_screen_control(7);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("B3");
                RCLCPP_INFO(this->get_logger(), "reach B2");
            } 
            else 
            {
                fly_to_target("B3"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to B6");
                    step_ = 12;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 12: // B6
            if (!hold_position_start_) {
                //发送B3货物编号,向串口屏n8.val赋值
                serial_screen_control(8);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("B6");
                RCLCPP_INFO(this->get_logger(), "reach B3");
            } 
            else 
            {
                fly_to_target("B6"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to B5");
                    step_ = 13;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 13: // B5
            if (!hold_position_start_) {
                //发送B6货物编号,向串口屏n11.val赋值
                serial_screen_control(11);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("B5");
                RCLCPP_INFO(this->get_logger(), "reach B6");
            } 
            else 
            {
                fly_to_target("B5"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to B4");
                    step_ = 14;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 14: // B4
            if (!hold_position_start_) {
                //发送B5货物编号,向串口屏n10.val赋值
                serial_screen_control(10);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                fly_to_target("B4");
                RCLCPP_INFO(this->get_logger(), "reach B5");
            } 
            else 
            {
                fly_to_target("B4"); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to B4");
                    step_ = 15;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 15: // 降落中间点1
            if (!hold_position_start_) {
                //发送B4货物编号,向串口屏n9.val赋值
                serial_screen_control(9);
                //激光笔指示
                control_laser_pointer(1,0.5);
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                publish_position(1.80, 0.20, 1.3);
                RCLCPP_INFO(this->get_logger(), "reach B4");
            } 
            else 
            {
                publish_position(1.80, 0.20, 1.3); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to step 16");  //转云台
                    step_ = 16;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            case 16: //降落点上方
            if (!hold_position_start_) {
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                publish_position(1.80, 0.20, 1.3);
                RCLCPP_INFO(this->get_logger(), "reach step 16");
            } 
            else 
            {
                publish_position(1.80, 0.20, 1.3); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to step 111");  //转云台
                    step_ = 111;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;
            //------------------------------------------------------ 


            //---------------------------任务2----------------------
            case 201: 
            if (!hold_position_start_) {
                hold_pisition_start_time_= this->now();
                hold_position_start_ = true;
                publish_position(1.80, 0.20, 1.3);
                RCLCPP_INFO(this->get_logger(), "reach B4");
            } 
            else 
            {
                publish_position(1.80, 0.20, 1.3); 
                auto elapsed = this->now() - hold_pisition_start_time_;
                if (elapsed.seconds() >= 5.0) {
                    RCLCPP_INFO(this->get_logger(), "go to B4");  //转云台
                    step_ = 202;
                    hold_position_start_ = false;  // 清除状态
                }
            }
            break;

            //------------------------------------------------------- 

            case 111://降落
                if (!hold_position_start_) {
                hold_pisition_start_time_ = this->now();
                hold_position_start_ = true;
                publish_position(0.0, -1.60, 0.21);
                    RCLCPP_INFO(this->get_logger(), "stard landing");
                }
                else { 

                publish_position(0.0, -1.60, 0.21);
                auto elapsed = this->now() - hold_pisition_start_time_;

                if (elapsed.seconds() >= 8.0) {
                    RCLCPP_INFO(this->get_logger(), "landed.");

                    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();

                    arm_req->value = false;

                    arming_client_->async_send_request(arm_req);
                    // 等待 3 秒，确保 PX4 收到命令
                    rclcpp::sleep_for(std::chrono::seconds(3));

                    RCLCPP_INFO(this->get_logger(), "Disarm request sent. Shutting down...");

                    rclcpp::shutdown();   // 关闭节点

                    }
                }
                break;

                
        }
            auto t2 = this->now();
            // RCLCPP_INFO(this->get_logger(), "%.2f ms", (t2 - t1).nanoseconds() / 1e6);
    }

    void handle_init_phase() {
            auto message = mavros_msgs::msg::PositionTarget();
        message.header.stamp = this->now();
        message.header.frame_id = "map";
        message.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        
        // 设置掩码只使用位置控制
        message.type_mask = 
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE|
            mavros_msgs::msg::PositionTarget::IGNORE_YAW;

        // 设置初始位置
        message.position.x = 0.0;
        message.position.y = 0.0;
        message.position.z = 0.12;  // 设置一个小的高度作为初始目标
        

        raw_pub_->publish(message);
/*
         // 发布固定位置 setpoint 保持 FCU 接受 OFFBOARD 模式
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 1.0; // 起飞目标高度
    pose_pub_->publish(pose); // 关键：>2Hz 持续发布
  */      
        // 模式未切换则切换
        if (current_state_.mode != "OFFBOARD") {
            if ((this->now() - last_request_time_).seconds() > 2.0) {
                //实际飞行需要注释掉！如果通过程序切offboard，飞机失控时遥控器将无法接管，注释掉程序会一直等待遥控器切入offboard

             //auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
             //mode_req->custom_mode = "OFFBOARD";
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
    //通过货物编号来指点飞行
    void fly_to_target(const std::string& position_label) {
        //定义位置对应坐标
        std::unordered_map<std::string, std::tuple<double, double, double>> positions = {
        {"A1", {0.0, 0.0, 1.3}}, {"A2", {1.0, 1.0, 1.3}}, {"A3", {0.0, 1.0, 1.3}},
        {"A4", {1.0, 0.0, 1.3}}, {"A5", {1.0, 1.0, 1.3}}, {"A6", {1.0, 1.0, 1.3}},

        {"B1", {2.0, 0.0, 1.3}}, {"B2", {2.0, 1.0, 1.3}}, {"B3", {2.0, 2.0, 1.3}},
        {"B4", {3.0, 0.0, 1.3}}, {"B5", {3.0, 1.0, 1.3}}, {"B6", {3.0, 2.0, 1.3}},

        {"C1", {4.0, 0.0, 1.3}}, {"C2", {4.0, 1.0, 1.3}}, {"C3", {4.0, 2.0, 1.3}},
        {"C4", {5.0, 0.0, 1.3}}, {"C5", {5.0, 1.0, 1.3}}, {"C6", {5.0, 2.0, 1.3}},

        {"D1", {6.0, 0.0, 1.3}}, {"D2", {6.0, 1.0, 1.3}}, {"D3", {6.0, 2.0, 1.3}},
        {"D4", {7.0, 0.0, 1.3}}, {"D5", {7.0, 1.0, 1.3}}, {"D6", {7.0, 2.0, 1.3}}
    };
        //判断编号是否有效
        if (positions.find(position_label) == positions.end()) {
        RCLCPP_WARN(this->get_logger(), "位置编号无效: %s", position_label.c_str());
        return;
    }

        auto target_pos = positions[position_label];

        auto message = mavros_msgs::msg::PositionTarget();
        message.header.stamp = this->now();
        message.header.frame_id = "map";
        message.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        
        // 设置掩码只使用速度控制
        message.type_mask = 
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW;

        
        message.position.x = std::get<0>(target_pos);
        message.position.y = std::get<1>(target_pos);
        message.position.z = std::get<2>(target_pos);
        //message.yaw = 0.0;

        // 发布位置
        raw_pub_->publish(message);
    }

    void publish_position(double px, double py, double pz) {
        auto message = mavros_msgs::msg::PositionTarget();
        message.header.stamp = this->now();
        message.header.frame_id = "map";
        message.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        
        // 设置掩码只使用速度控制
        message.type_mask = 
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW;

        // 速度限幅
        message.position.x = px;
        message.position.y = py;
        message.position.z = pz;
        //message.yaw = 0.0;

        // 发布速度命令
        raw_pub_->publish(message);
    }

    //步进电机控制函数，直接输入要转动的角度
void control_angle(int angle) {
    std_msgs::msg::Float64 msg;
    msg.data = static_cast<double>(angle);  // 转换为 double 类型
    cam_angle_pub_->publish(msg);  // 发布消息

    RCLCPP_INFO(this->get_logger(), "usb cam turned to %d", angle);
}


    void control_laser_pointer(int flag,int time) {
        auto message = std_msgs::msg::Float64MultiArray();

        // 激光状态 1 表示点亮，0 表示熄灭
        message.data.push_back(flag);

        // 激光点亮的时间
        message.data.push_back(time);  

        // 发布消息
        laser_pointer_pub_->publish(message);
   
    RCLCPP_INFO(this->get_logger(), "laser pointer on");
}

 void serial_screen_control(int number) {
    // 创建一个消息对象
    auto message = std_msgs::msg::String();

    // 构造消息内容，格式为 "n{number}.val={qr_data}"
    message.data = "n" + std::to_string(number) + ".val=" + std::to_string(qr_data_);

    // 发布消息到 /serial_screen_command
    serial_screen_pub_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Published message: %s", message.data.c_str());
}

    
    int step_;
    int flag_;

    rclcpp::Time last_request_time_;

    rclcpp::Time start_time_;
    //舵机控制时间延时
    rclcpp::Time servo_action_start_time_;
    //悬停时间
    rclcpp::Time hold_pisition_start_time_;

    std::string task_msg_;

    int qr_data_;
    //二维码扫描标志位
    bool qr_scan_started_ = false;

    bool hold_position_start_ = false;

    bool pid_enabled_ = false;
    
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_vel_;

    PID pid_x_, pid_y_, pid_z_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_subscription_;
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr qr_data_subscription_;
   
    //  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    //  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr raw_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cam_angle_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr laser_pointer_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_screen_pub_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

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
