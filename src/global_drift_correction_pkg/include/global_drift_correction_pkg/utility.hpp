#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


class Utility
{
    private:
        typedef pcl::PointXYZI PointType;

    public:
        static void cloudPublish(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_, \
            pcl::PointCloud<PointType>::Ptr pointcloud)
        {
            sensor_msgs::msg::PointCloud2 cloud_temp;
            pcl::toROSMsg(*pointcloud, cloud_temp);
            // cloud_temp.header.stamp = now();
            cloud_temp.header.frame_id = "map";
            if(publisher_->get_subscription_count() != 0)
            {
                publisher_->publish(cloud_temp);
            }
        }
};