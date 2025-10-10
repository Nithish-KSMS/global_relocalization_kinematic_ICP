#include "global_drift_correction_pkg/utility.hpp"
#include "global_drift_correction_pkg/globalLocalize.hpp"


GlobalRelocalization::GlobalRelocalization() : Node("global_relocalization")
{
    RCLCPP_INFO(this->get_logger() ,"Starting Global Relocalization Node");

    cloudGlobalLoad();
    
    pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 10); 
    pcd_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GlobalRelocalization::pcd_publish_timercallback, this));
    
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/dummy/livox/lidar", 10, std::bind(&GlobalRelocalization::pointcloudCallback, this, std::placeholders::_1));

    // tf2::Duration cache_time = tf2::Duration(20);
    // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), cache_time);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);  
    
    transformed_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/odom_frame_transformed_pointcloud", 10);

    for(int i=0; i<6; i++)
    {
        transformOdomToWorld[i] = 0.0;
        transformInTheWorld[i] = 0.0;
        transformToBeMapped[i] = 0.0;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    initializedFlag = NonInitialized;
    
    initial_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&GlobalRelocalization::initialposeCallback, this, std::placeholders::_1));

    initial_pose_received = 0;
}

void GlobalRelocalization::cloudGlobalLoad()
{
    map_path = "/home/nithish/global_relocalization_ws/RTAB_Map_PCD/cloud.pcd";
    pcl::io::loadPCDFile<PointType>(map_path, *cloudGlobalMap);
    std::cout << "The size of the global pointcloud map BEFORE filtering: " << cloudGlobalMap->points.size() << std::endl;
    
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    float mappingSurfLeafSize = 0.3;
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setInputCloud(cloudGlobalMap);
    downSizeFilterICP.filter(*cloudGlobalMapFiltered);
    std::cout << "The size of the global pointcloud map AFTER filtering: " << cloudGlobalMapFiltered->points.size() << std::endl;

}

void GlobalRelocalization::pcd_publish_timercallback()
{
    // ADD the timeLaserInfoStamp later
    Utility::cloudPublish(pcd_publisher_, cloudGlobalMap);
}


void GlobalRelocalization::tfBroadcaster(const float (&transformOdomToWorld)[6], const builtin_interfaces::msg::Time &header_stamp)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = header_stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "dummy_odom";

    t.transform.translation.x = transformOdomToWorld[3];
    t.transform.translation.y = transformOdomToWorld[4];
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0,0,transformOdomToWorld[2]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

PointTypePose GlobalRelocalization::trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll  = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw   = transformIn[2];
    return thisPose6D;
}

Eigen::Affine3f GlobalRelocalization::pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

void GlobalRelocalization::pointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
{
    // auto start1 = std::chrono::high_resolution_clock::now();

    // auto end1 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed_seconds1 = end1 - start1;
    // std::cout << "Time taken for Pointcloud data TF conversion: " << elapsed_seconds1.count() << " seconds" << std::endl;
    if(initializedFlag == NonInitialized && initial_pose_received == 1)
    {
        initial_pose_received = 0;
        bool pointcloud_tf_success = pcl_ros::transformPointCloud("dummy_base_link", *input_cloud, transformed_cloud, *tf_buffer_);
        if(pointcloud_tf_success)
        {
            // RCLCPP_INFO(this->get_logger(), "Pointcloud transformed successfully to %s frame", "odom");
            transformed_cloud_publisher_->publish(transformed_cloud);
            scanMatchInitial(transformed_cloud);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Pointcloud transformation to %s frame FAILED. \t SKIPPING Map to Odom correction", "odom");
            return;
        }
    }
    else if(initializedFlag == Initializing)
    {
        std::cout << "Offer A New Guess Please " << std::endl;
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
    else if(initializedFlag == Initialized)
    {
        // std::cout << "Inside Scan Match Global block" << std::endl;
        bool pointcloud_tf_success = pcl_ros::transformPointCloud("dummy_odom", *input_cloud, transformed_cloud, *tf_buffer_);
        if(pointcloud_tf_success)
        {
            // RCLCPP_INFO(this->get_logger(), "Pointcloud transformed successfully to %s frame", "odom");
            transformed_cloud_publisher_->publish(transformed_cloud);
            scanMatchGlobal(transformed_cloud);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Pointcloud transformation to %s frame FAILED. \t SKIPPING Map to Odom correction", "odom");
            return;
        }
    }
}

void GlobalRelocalization::scanMatchInitial(sensor_msgs::msg::PointCloud2 &transformed_cloud)
{
    pcl::fromROSMsg(transformed_cloud, *transformed_cloud_pcl);

    pcl::PointCloud<PointType>::Ptr filtered_cloud{new pcl::PointCloud<PointType>};
    pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (transformed_cloud_pcl);
    approximate_voxel_filter.filter (*filtered_cloud);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(35);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    icp.setInputSource(filtered_cloud);
    icp.setInputTarget(cloudGlobalMapFiltered);
    pcl::PointCloud<PointType>::Ptr unused_result_0{new pcl::PointCloud<PointType>};

    PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
    Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);

    icp.align(*unused_result_0, T_thisPose6DInWorld.matrix());

    std::cout << "Transformation from base_link to map BEFORE initialization" << std::endl;
    std::cout << "x :" << transformInTheWorld[3] << ", y:" <<transformInTheWorld[4] << " yaw: " << transformInTheWorld[2]; 

    // Obtain transformation from map to base_link
    Eigen::Affine3f T_thisPose6DInMap;
    T_thisPose6DInMap = icp.getFinalTransformation();
    float x_g, y_g, z_g, R_g, P_g, Y_g;
    pcl::getTranslationAndEulerAngles (T_thisPose6DInMap, x_g, y_g, z_g, R_g, P_g, Y_g);
    transformInTheWorld[0] = R_g;
    transformInTheWorld[1] = P_g;
    transformInTheWorld[2] = Y_g;
    transformInTheWorld[3] = x_g;
    transformInTheWorld[4] = y_g;
    transformInTheWorld[5] = z_g;

    std::cout << "Transformation from base_link to map AFTER initialization" << std::endl;
    std::cout << "x :" << transformInTheWorld[3] << ", y:" <<transformInTheWorld[4] << " yaw: " << transformInTheWorld[2];

    // Calculate transformation from map to odom
    PointTypePose thisPose6DInOdom = trans2PointTypePose(transformToBeMapped);
    Eigen::Affine3f T_thisPose6DInOdom = pclPointToAffine3f(thisPose6DInOdom);

    Eigen::Affine3f transOdomToMap = T_thisPose6DInMap * T_thisPose6DInOdom.inverse();
    float deltax, deltay, deltaz, deltaR, deltaP, deltaY;
    pcl::getTranslationAndEulerAngles (transOdomToMap, deltax, deltay, deltaz, deltaR, deltaP, deltaY);

    transformOdomToWorld[0] = deltaR;
    transformOdomToWorld[1] = deltaP;
    transformOdomToWorld[2] = deltaY;
    transformOdomToWorld[3] = deltax;
    transformOdomToWorld[4] = deltay;
    transformOdomToWorld[5] = deltaz;

    if(icp.hasConverged())
    {
        initializedFlag = Initialized;
        RCLCPP_INFO(this->get_logger(), "Initialization successful!");
        tfBroadcaster(transformOdomToWorld, transformed_cloud.header.stamp);
        RCLCPP_INFO(this->get_logger(), "Starting Global Drift Correction");
    }
    else
    {
        initializedFlag = Initializing;
        RCLCPP_INFO(this->get_logger(), "Initialization FAILED");
    }
}

void GlobalRelocalization::scanMatchGlobal(sensor_msgs::msg::PointCloud2 &transformed_cloud)
{
    pcl::fromROSMsg(transformed_cloud, *transformed_cloud_pcl);

    pcl::PointCloud<PointType>::Ptr filtered_cloud{new pcl::PointCloud<PointType>};
    pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (transformed_cloud_pcl);
    approximate_voxel_filter.filter (*filtered_cloud);
    // std::cout << "Filtered cloud contains " << filtered_cloud->size () << " data points from real-time pointcloud data" << std::endl;

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0); // default 1.0
    ndt.setMaximumIterations(10);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(35);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    Eigen::Affine3f transodomToWorld_init = pcl::getTransformation(transformOdomToWorld[3],\
         transformOdomToWorld[4],transformOdomToWorld[5],\
         transformOdomToWorld[0], transformOdomToWorld[1],\
         transformOdomToWorld[2]);
    Eigen::Matrix4f matricInitGuess = transodomToWorld_init.matrix();

    // auto start2 = std::chrono::high_resolution_clock::now();

    // // ndt.setInputSource(GlobalRelocalization::transformed_cloud_pcl);
    // ndt.setInputSource(filtered_cloud);
    // ndt.setInputTarget(GlobalRelocalization::cloudGlobalMapFiltered);
    // pcl::PointCloud<PointType>::Ptr unused_result_0(new pcl::PointCloud<PointType>());
    // std::cout<< "Inside NDT Timer block" << std::endl;
    // ndt.align(*unused_result_0, matricInitGuess);

    // auto end2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed_seconds2 = end2 - start2;
    // std::cout << "Time taken for NDT Scan Matching: " << elapsed_seconds2.count() << " seconds" << std::endl;


    // auto start3 = std::chrono::high_resolution_clock::now();
    // icp.setInputSource(GlobalRelocalization::transformed_cloud_pcl);
    icp.setInputSource(filtered_cloud);
    icp.setInputTarget(cloudGlobalMapFiltered);
    pcl::PointCloud<PointType>::Ptr unused_result_1{new pcl::PointCloud<PointType>};
    // icp.align(*unused_result_1, ndt.getFinalTransformation());
    icp.align(*unused_result_1, matricInitGuess);

    // auto end3 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed_seconds3 = end3 - start3;
    // std::cout << "Time taken for ICP Scan Matching: " << elapsed_seconds3.count() << " seconds" << std::endl;

    Eigen::Affine3f transformOdomToWorldNew;
    transformOdomToWorldNew = icp.getFinalTransformation();
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles (transformOdomToWorldNew, x, y, z, roll, pitch, yaw);

    transformOdomToWorld[0] = roll;
    transformOdomToWorld[1] = pitch; 
    transformOdomToWorld[2] = yaw;
    transformOdomToWorld[3] = x;
    transformOdomToWorld[4] = y;
    transformOdomToWorld[5] = z;

    // std::cout << "New Map to Odom Transform" << std::endl;
    // std::cout << "x: " << x << "\t" << "y: " << y << "\t" << "yaw: " << yaw << std::endl;

    tfBroadcaster(transformOdomToWorld, transformed_cloud.header.stamp);
}

void GlobalRelocalization::initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg)
{
    if(initializedFlag == Initialized)
    {
        return;
    }

    float x_global = pose_msg->pose.pose.position.x;
    float y_global = pose_msg->pose.pose.position.y;
    float z_global = pose_msg->pose.pose.position.z;

    tf2::Quaternion q_global;
    double roll_global; double pitch_global; double yaw_global;

    q_global.setX(pose_msg->pose.pose.orientation.x);
    q_global.setY(pose_msg->pose.pose.orientation.y);
    q_global.setZ(pose_msg->pose.pose.orientation.z);
    q_global.setW(pose_msg->pose.pose.orientation.w);
    
    tf2::Matrix3x3(q_global).getRPY(roll_global, pitch_global, yaw_global);
    
    // TF from map to base_link
    transformInTheWorld[0] = roll_global;
    transformInTheWorld[1] = pitch_global;
    transformInTheWorld[2] = yaw_global;
    transformInTheWorld[3] = x_global;
    transformInTheWorld[4] = y_global;
    transformInTheWorld[5] = z_global;

    PointTypePose thisPose6DInWorld = trans2PointTypePose(transformInTheWorld);
    Eigen::Affine3f T_thisPose6DInWorld = pclPointToAffine3f(thisPose6DInWorld);

    geometry_msgs::msg::TransformStamped odom_to_baselink_tf;

    try
    {
        odom_to_baselink_tf = tf_buffer_->lookupTransform("dummy_odom", "dummy_base_link", tf2::TimePointZero); 
    }
    catch (const tf2::TransformException &Ex)
    {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "odom", "base_link", Ex.what());
        return;
    }

    float x_odom = odom_to_baselink_tf.transform.translation.x;
    float y_odom = odom_to_baselink_tf.transform.translation.y;
    float z_odom = odom_to_baselink_tf.transform.translation.z;

    tf2::Quaternion q_odom;
    double roll_odom; double pitch_odom; double yaw_odom;

    q_odom.setX(odom_to_baselink_tf.transform.rotation.x);
    q_odom.setY(odom_to_baselink_tf.transform.rotation.y);
    q_odom.setZ(odom_to_baselink_tf.transform.rotation.z);
    q_odom.setW(odom_to_baselink_tf.transform.rotation.w);

    tf2::Matrix3x3(q_odom).getRPY(roll_odom, pitch_odom, yaw_odom);

    transformToBeMapped[0] = roll_odom;
    transformToBeMapped[1] = pitch_odom;
    transformToBeMapped[2] = yaw_odom;
    transformToBeMapped[3] = x_odom;
    transformToBeMapped[4] = y_odom;
    transformToBeMapped[5] = z_odom;

    std::cout << "Transformation from odom frame to base_link: x:" << x_odom << ", y: " << y_odom << ", yaw: " << yaw_odom << std::endl;

    initial_pose_received = 1;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<GlobalRelocalization>());

    rclcpp::shutdown();

    return 0;
}