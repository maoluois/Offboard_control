#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

class LaserTransferNode : public rclcpp::Node {
public:
    LaserTransferNode()
        : Node("cartographer_laser_transfer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        
        // 发布到 MAVROS 的 /mavros/vision_pose/pose 话题
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10);
        
        // 使用定时器（30Hz）定期执行 cartographer 方法
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), 
                                         std::bind(&LaserTransferNode::cartographer, this));
    }

private:
    void cartographer() {
        try {
            // 获取 base_link 在 map 参考坐标系下的位姿
            geometry_msgs::msg::TransformStamped tfstamped = 
                tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            // 组装 PoseStamped 消息
            geometry_msgs::msg::PoseStamped local_pose;
            local_pose.header.stamp = this->get_clock()->now();
            local_pose.header.frame_id = "map";
            local_pose.pose.position.x = tfstamped.transform.translation.x;
            local_pose.pose.position.y = tfstamped.transform.translation.y;
            local_pose.pose.position.z = tfstamped.transform.translation.z;  // 直接使用 TF 变换中的 Z 轴数据
            local_pose.pose.orientation = tfstamped.transform.rotation;

            // 发布位姿消息
            pose_pub_->publish(local_pose);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserTransferNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
