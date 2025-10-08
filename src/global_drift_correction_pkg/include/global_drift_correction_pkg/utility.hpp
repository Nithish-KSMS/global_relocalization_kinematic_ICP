#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl_ros/transforms.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>


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