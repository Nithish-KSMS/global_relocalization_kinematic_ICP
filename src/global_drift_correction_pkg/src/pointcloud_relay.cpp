#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudRelay : public rclcpp::Node
{
    public:
        PointCloudRelay() : Node("point_cloud_relay")
        {
            RCLCPP_INFO(this->get_logger(), "Starting PointCloud Relay Node");
            pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10, \
                                std::bind(&PointCloudRelay::point_cloud_relay_callback, this, std::placeholders::_1));

            relayed_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dummy/livox/lidar", 10);
        }

    private:
        void point_cloud_relay_callback(sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg)
        {
            relayed_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*pointcloud_msg);
            relayed_pointcloud->header.frame_id = "dummy_livox_frame";
            relayed_pointcloud_publisher_->publish(*relayed_pointcloud);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
        sensor_msgs::msg::PointCloud2::SharedPtr relayed_pointcloud;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr relayed_pointcloud_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudRelay>());
    rclcpp::shutdown();
    return 0;
}