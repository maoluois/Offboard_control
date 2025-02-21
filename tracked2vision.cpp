#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

class PoseForwarder : public rclcpp::Node
{
public:
  PoseForwarder()
      : Node("pose_forwarder")
  {
    // 创建发布者和订阅者
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "tracked_pose", 10, std::bind(&PoseForwarder::pose_callback, this, std::placeholders::_1));

    vision_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
  }

private:
  // 回调函数，用于接收tracked_pose话题的PoseStamped消息并转发
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 将接收到的PoseStamped消息发布到/mavros/vision_pose/pose话题
    vision_pose_pub_->publish(*msg);
    // RCLCPP_INFO(this->get_logger(), "Forwarded Pose to /mavros/vision_pose/pose");
  }

  // ROS2订阅者和发布者
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseForwarder>());
  rclcpp::shutdown();
  return 0;
}
