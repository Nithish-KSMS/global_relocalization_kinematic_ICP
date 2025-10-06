#include "global_drift_correction_pkg/utility.hpp"
#include "global_drift_correction_pkg/globalLocalize.hpp"



// A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment


POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
(float, x, x) (float, y, y)
(float, z, z) (float, intensity, intensity)
(float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
(double, time, time))

typedef PointXYZIRPYT  PointTypePose;

GlobalRelocalization::GlobalRelocalization() : Node("global_relocalization")
{
    RCLCPP_INFO(this->get_logger() ,"Starting global relocalization for drift correction");

    cloudGlobalLoad();
    
    GlobalRelocalization::pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 10); 
    GlobalRelocalization::pcd_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GlobalRelocalization::pcd_publish_timercallback, this));
    
    GlobalRelocalization::pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10, std::bind(&GlobalRelocalization::pointcloudCallback, this, std::placeholders::_1));

    GlobalRelocalization::tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    GlobalRelocalization::tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*GlobalRelocalization::tf_buffer_);  
    
    GlobalRelocalization::transformed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/odom_frame_transformed_pointcloud", 10);

    for(int i=0; i<6; i++)
    {
        GlobalRelocalization::transformOdomToWorld[i] = 0.0;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void GlobalRelocalization::cloudGlobalLoad()
{
    GlobalRelocalization::map_path = "/home/nithish/global_relocalization_ws/RTAB_Map_PCD/cloud.pcd";
    pcl::io::loadPCDFile<PointType>(GlobalRelocalization::map_path, *GlobalRelocalization::cloudGlobalMap);
    std::cout << "The size of the global pointcloud map BEFORE filtering: " << cloudGlobalMap->points.size() << std::endl;
    
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    float mappingSurfLeafSize = 0.3;
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setInputCloud(GlobalRelocalization::cloudGlobalMap);
    downSizeFilterICP.filter(*GlobalRelocalization::cloudGlobalMapFiltered);
    std::cout << "The size of the global pointcloud map AFTER filtering: " << cloudGlobalMapFiltered->points.size() << std::endl;

}

void GlobalRelocalization::pcd_publish_timercallback()
{
    // ADD the timeLaserInfoStamp later
    Utility::cloudPublish(GlobalRelocalization::pcd_publisher_, GlobalRelocalization::cloudGlobalMap);
}

void GlobalRelocalization::pointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
{
    auto start1 = std::chrono::high_resolution_clock::now();
    
    bool pointcloud_tf_success = pcl_ros::transformPointCloud("odom", *input_cloud, GlobalRelocalization::transformed_cloud, *GlobalRelocalization::tf_buffer_);
    if(pointcloud_tf_success)
    {
        // RCLCPP_INFO(this->get_logger(), "Pointcloud transformed successfully to %s frame", "odom");
        GlobalRelocalization::transformed_cloud_publisher_->publish(GlobalRelocalization::transformed_cloud);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Pointcloud transformation to %s frame FAILED. \t SKIPPING Map to Odom correction", "odom");
        return;
    }

    auto end1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
    std::cout << "Time taken for Pointcloud data TF conversion: " << elapsed_seconds1.count() << " seconds" << std::endl;

    pcl::fromROSMsg(transformed_cloud, *GlobalRelocalization::transformed_cloud_pcl);

    pcl::PointCloud<PointType>::Ptr filtered_cloud{new pcl::PointCloud<PointType>};
    pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (GlobalRelocalization::transformed_cloud_pcl);
    approximate_voxel_filter.filter (*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size () << " data points from room_scan2.pcd" << std::endl;

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.2);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(10);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(35);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    Eigen::Affine3f transodomToWorld_init = pcl::getTransformation(GlobalRelocalization::transformOdomToWorld[3],\
         GlobalRelocalization::transformOdomToWorld[4],GlobalRelocalization::transformOdomToWorld[5],\
         GlobalRelocalization::transformOdomToWorld[0], GlobalRelocalization::transformOdomToWorld[1],\
         GlobalRelocalization::transformOdomToWorld[2]);
    Eigen::Matrix4f matricInitGuess = transodomToWorld_init.matrix();

    // auto start2 = std::chrono::high_resolution_clock::now();

    // // ndt.setInputSource(GlobalRelocalization::transformed_cloud_pcl);
    // ndt.setInputSource(filtered_cloud);
    // ndt.setInputTarget(GlobalRelocalization::cloudGlobalMapFiltered);
    // pcl::PointCloud<PointType>::Ptr unused_result_0{new pcl::PointCloud<PointType>};
    // ndt.align(*unused_result_0, matricInitGuess);

    // auto end2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
    // std::cout << "Time taken for NDT Scan Matching: " << elapsed_seconds2.count() << " seconds" << std::endl;


    auto start3 = std::chrono::high_resolution_clock::now();
    // icp.setInputSource(GlobalRelocalization::transformed_cloud_pcl);
    icp.setInputSource(filtered_cloud);
    icp.setInputTarget(GlobalRelocalization::cloudGlobalMapFiltered);
    pcl::PointCloud<PointType>::Ptr unused_result_1{new pcl::PointCloud<PointType>};
    // icp.align(*unused_result_1, ndt.getFinalTransformation());
    icp.align(*unused_result_1, matricInitGuess);

    auto end3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds3 = end3 - start3;
    std::cout << "Time taken for ICP Scan Matching: " << elapsed_seconds3.count() << " seconds" << std::endl;

    Eigen::Affine3f transformOdomToWorldNew;
    transformOdomToWorldNew = icp.getFinalTransformation();
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles (transformOdomToWorldNew, x, y, z, roll, pitch, yaw);

    GlobalRelocalization::transformOdomToWorld[0] = roll;
    GlobalRelocalization::transformOdomToWorld[1] = pitch;
    GlobalRelocalization::transformOdomToWorld[2] = yaw;
    GlobalRelocalization::transformOdomToWorld[3] = x;
    GlobalRelocalization::transformOdomToWorld[4] = y;
    GlobalRelocalization::transformOdomToWorld[5] = z;

    std::cout << "New Map to Odom Transform" << std::endl;
    std::cout << "x: " << x << "\t" << "y: " << y << "\t" << "yaw: " << yaw << std::endl;
    
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = input_cloud->header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";

    t.transform.translation.x = GlobalRelocalization::transformOdomToWorld[3];
    t.transform.translation.y = GlobalRelocalization::transformOdomToWorld[4];
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0,0,GlobalRelocalization::transformOdomToWorld[2]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<GlobalRelocalization>());

    rclcpp::shutdown();

    return 0;
}