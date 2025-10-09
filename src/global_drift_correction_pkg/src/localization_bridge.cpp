#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"


class LocalizationBridge : public rclcpp::Node
{
    public:
        LocalizationBridge():Node("kinematic_icp_localization_bridge")
        {
            RCLCPP_INFO(this->get_logger(), "Starting LocalizationBridge Node");
            kinematic_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/kinematic_icp/lidar_odometry", \
                                                10, std::bind(&LocalizationBridge::kinematic_tf_callback, this, std::placeholders::_1));

            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

    private:
        void kinematic_tf_callback(nav_msgs::msg::Odometry::SharedPtr kinematic_icp_tf_msg)
        {
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = kinematic_icp_tf_msg->header.stamp;
            // t.header.frame_id = kinematic_icp_tf_msg->header.frame_id;
            t.header.frame_id = "dummy_odom";
            t.child_frame_id = "dummy_base_link";

            t.transform.translation.x = kinematic_icp_tf_msg->pose.pose.position.x;
            t.transform.translation.y = kinematic_icp_tf_msg->pose.pose.position.y;
            t.transform.translation.z = kinematic_icp_tf_msg->pose.pose.position.z;

            t.transform.rotation.x = kinematic_icp_tf_msg->pose.pose.orientation.x;
            t.transform.rotation.y = kinematic_icp_tf_msg->pose.pose.orientation.y;
            t.transform.rotation.z = kinematic_icp_tf_msg->pose.pose.orientation.z;
            t.transform.rotation.w = kinematic_icp_tf_msg->pose.pose.orientation.w;

            tf_broadcaster_->sendTransform(t);
        }
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_odom_subscriber_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;      
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationBridge>());
    rclcpp::shutdown();
    return 0;
}