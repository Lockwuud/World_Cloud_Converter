/*
 * @FilePath     : /src/World_Cloud_Converter/include/parameters.hpp
 * @Description  :  
 * @Author       : hejia 2736463842@qq.com
 * @Version      : 0.0.1
 * @LastEditors  : hejia 2736463842@qq.com
 * @LastEditTime : 2025-03-02 19:50:44
 * @Copyright    : G AUTOMOBILE RESEARCH INSTITUTE CO.,LTD Copyright (c) 2025.
*/
#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <thread>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cuda_runtime.h>

tf2_ros::Buffer buffer;

ros::Publisher pub;
ros::Publisher pub_without_edge;

pcl::PointCloud<pcl::PointXYZ> edgeCloud;

std::string cloud_topic_1, cloud_topic_2;

float ed_left, ed_right, ed_ceil, ed_floor, ed_resolution;
float passthrough_z_max, passthrough_z_min;
float half_width, half_height;

#endif