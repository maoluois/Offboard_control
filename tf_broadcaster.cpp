#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster()
            : Node("tf_broadcaster_node")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&TFBroadcaster::broadcast_transform, this));
    }

private:
    void broadcast_transform()
    {
        // NEU坐标系调整（假设原始坐标系为XYZ对应北东天）
        // 1. odom -> base_link 转换
        geometry_msgs::msg::TransformStamped odom_to_base_link;
        odom_to_base_link.header.stamp = this->get_clock()->now();
        odom_to_base_link.header.frame_id = "odom";
        odom_to_base_link.child_frame_id = "base_link";

        // 坐标系调整（根据实际传感器数据方向可能需要修改）
        tf2::Quaternion q_neu;
        q_neu.setRPY(0, 0, M_PI); // 示例旋转调整，具体数值需根据实际传感器方向确定
        odom_to_base_link.transform.rotation.x = q_neu.x();
        odom_to_base_link.transform.rotation.y = q_neu.y();
        odom_to_base_link.transform.rotation.z = q_neu.z();
        odom_to_base_link.transform.rotation.w = q_neu.w();

        broadcaster_->sendTransform(odom_to_base_link);

        // 2. base_link -> laser 转换（仅Z轴偏移）
        geometry_msgs::msg::TransformStamped base_link_to_laser;
        base_link_to_laser.header.stamp = this->get_clock()->now();
        base_link_to_laser.header.frame_id = "base_link";
        base_link_to_laser.child_frame_id = "laser";

        // 设置平移（假设激光雷达在base_link正上方0.2米）
        base_link_to_laser.transform.translation.x = 0.0;
        base_link_to_laser.transform.translation.y = 0.0;
        base_link_to_laser.transform.translation.z = 0.088; // 高度差

        // 保持旋转一致
        tf2::Quaternion q_no_rotation;
        q_no_rotation.setRPY(0, 0, 0);
        base_link_to_laser.transform.rotation.x = q_no_rotation.x();
        base_link_to_laser.transform.rotation.y = q_no_rotation.y();
        base_link_to_laser.transform.rotation.z = q_no_rotation.z();
        base_link_to_laser.transform.rotation.w = q_no_rotation.w();

        broadcaster_->sendTransform(base_link_to_laser);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
