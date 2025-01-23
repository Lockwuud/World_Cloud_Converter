/*
 * @Author: hejia 2736463842@qq.com
 * @Date: 2025-01-22 11:57:29
 * @LastEditors: hejia 2736463842@qq.com
 * @LastEditTime: 2025-01-22 15:17:48
 * @FilePath: /src/world_cloud_converter/src/map_node.cpp
 * @Description: 
 */
#include "rog_map/rog_map.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    /* 1. Creat a ROGMap ptr*/
    rog_map::ROGMap::Ptr rog_map_ptr = std::make_shared<rog_map::ROGMap>(nh);

    /* Publisher and subcriber */
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();

    ros::waitForShutdown();
    return 0;
}
