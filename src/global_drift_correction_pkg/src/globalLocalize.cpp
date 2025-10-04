#include "global_drift_correction_pkg/utility.hpp"
#include "global_drift_correction_pkg/globalLocalize.hpp"



GlobalRelocalization::GlobalRelocalization() : Node("global_relocalization")
    {
        RCLCPP_INFO(this->get_logger() ,"Starting global relocalization for drift correction");
        GlobalRelocalization::pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map", 10); 
        GlobalRelocalization::pcd_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GlobalRelocalization::pcd_publish_timercallback, this));
        cloudGlobalLoad();
    }

void GlobalRelocalization::cloudGlobalLoad()
    {
        GlobalRelocalization::map_path = "/home/nithish/lio_sam_relocalization_gaochao_hit/src/LIO-SAM_based_relocalization/results/lvi_sam_pcd/GlobalMap.pcd";
        pcl::io::loadPCDFile<PointType>(GlobalRelocalization::map_path, *GlobalRelocalization::cloudGlobalMap);
        std::cout << "The size of the global pointcloud map: " << cloudGlobalMap->points.size() << std::endl;
    }

void GlobalRelocalization::pcd_publish_timercallback()
    {
        // ADD the timeLaserInfoStamp later
        Utility::cloudPublish(GlobalRelocalization::pcd_publisher_, GlobalRelocalization::cloudGlobalMap);
    }




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<GlobalRelocalization>());

    rclcpp::shutdown();

    return 0;
}