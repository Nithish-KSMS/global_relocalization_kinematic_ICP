#include "global_drift_correction_pkg/utility.hpp"

class GlobalRelocalization : public rclcpp::Node
{
    public:
        GlobalRelocalization();

    private:
        void cloudGlobalLoad();
        void pcd_publish_timercallback();

        typedef pcl::PointXYZI PointType;

        // Publisher and Timer definition for Global PCD
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
        rclcpp::TimerBase::SharedPtr pcd_publish_timer_;
        
        // Variable for storing the PCD Map path
        std::string map_path; 

        // Variable for storing PCD file
        pcl::PointCloud<PointType>::Ptr cloudGlobalMap{new pcl::PointCloud<PointType>};  

        // Variable to hold time stamp; Used while publishing pointclouds
        rclcpp::Time timeLaserInfoStamp;
};