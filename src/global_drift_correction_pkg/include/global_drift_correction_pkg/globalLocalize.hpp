#include "global_drift_correction_pkg/utility.hpp"

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


class GlobalRelocalization : public rclcpp::Node
{
    public:
        GlobalRelocalization();

    private:
        void cloudGlobalLoad();
        void pcd_publish_timercallback();
        void pointcloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr);
        void tfBroadcaster(const float (&transformOdomToWorld)[6], const builtin_interfaces::msg::Time &header_stamp);
        void scanMatchGlobal(sensor_msgs::msg::PointCloud2 &transformed_cloud);
        void initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);
        PointTypePose trans2PointTypePose(float transformIn[]);
        Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);
        void scanMatchInitial(sensor_msgs::msg::PointCloud2 &transformed_cloud);

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
        
        // variable to hold the odom to map TF
        float transformOdomToWorld[6];

        // variable to hold the map to base_link TF
        float transformInTheWorld[6];

        // variable to hold the odom to base_link TF
        float transformToBeMapped[6];
        
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        
        enum InitializedFlag
        {
            NonInitialized,
            Initializing,
            Initialized
        };
        InitializedFlag initializedFlag;
        
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscriber_;

        bool initial_pose_received;
};