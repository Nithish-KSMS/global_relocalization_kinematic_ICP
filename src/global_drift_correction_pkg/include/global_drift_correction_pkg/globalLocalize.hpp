#include "global_drift_correction_pkg/utility.hpp"

class GlobalRelocalization : public rclcpp::Node
{
    public:
        GlobalRelocalization();

    private:
        void cloudGlobalLoad();
        void pcd_publish_timercallback();
        void pointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr);

        typedef pcl::PointXYZI PointType;

        // Publisher and Timer definition for Global PCD
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
        rclcpp::TimerBase::SharedPtr pcd_publish_timer_;
        
        // Variable for storing the PCD Map path
        std::string map_path; 

        // Variable for storing PCD file
        pcl::PointCloud<PointType>::Ptr cloudGlobalMap{new pcl::PointCloud<PointType>};  

        // Variable for storing filtered PCD Map
        pcl::PointCloud<PointType>::Ptr cloudGlobalMapFiltered{new pcl::PointCloud<PointType>};

        // Variable to hold time stamp; Used while publishing pointclouds
        rclcpp::Time timeLaserInfoStamp;

        // Subscriber for Lidar Pointcloud data
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;

        // pcl variable incoming Lidar Pointcloud data
        pcl::PointCloud<PointType>::Ptr transformed_cloud_pcl{new pcl::PointCloud<PointType>};
        
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        geometry_msgs::msg::TransformStamped t;

        // Pointcloud transformed to odom frame
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        
        // Publisher for transformed pointcloud
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_publisher_;
        
        // variable to hold the odom the map TF
        float transformOdomToWorld[6];

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};