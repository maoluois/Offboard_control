#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

#include <chrono>

using namespace std::chrono_literals;  // This allows the use of the 'ms' suffix

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&OffboardControl::state_callback, this, std::placeholders::_1));
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "mavros/setpoint_position/local", 10);
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");

        timer_ = this->create_wall_timer(
            100ms, std::bind(&OffboardControl::timer_callback, this));

        pose_.pose.position.x = 0.0;
        pose_.pose.position.y = 0.0;
        pose_.pose.position.z = 2.0;
    }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void timer_callback()
    {
        if (current_state_.mode != "OFFBOARD")
        {
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(request);
        }

        if (!current_state_.armed)
        {
            auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            request->value = true;
            arming_client_->async_send_request(request);
        }

        local_pos_pub_->publish(pose_);
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
