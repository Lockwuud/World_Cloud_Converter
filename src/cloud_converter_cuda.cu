/*
 * @Author: hejia 2736463842@qq.com
 * @Date: 2025-02-02 19:39:52
 * @LastEditors: hejia 2736463842@qq.com
 * @LastEditTime: 2025-02-02 21:10:36
 * @FilePath: /ego-planner-swarm/src/World_Cloud_Converter/src/cloud_process.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by hejia 2736463842@qq.com, All Rights Reserved. 
 */
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

#include "tictoc.hpp"


tf2_ros::Buffer buffer;

ros::Publisher pub;
ros::Publisher pub_without_edge;

pcl::PointCloud<pcl::PointXYZ> edgeCloud;

float ed_left, ed_right, ed_ceil, ed_floor, ed_resolution, self_half_height, self_half_width, passthrough_z_max, passthrough_z_min;

bool en_passthrough_z = false;

pcl::PointCloud<pcl::PointXYZ> generateRectanglePointCloud(float edge_left, float edge_right, float edge_floor, float edge_ceil, float resolution)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    int num_points_x = fabsf(edge_ceil - edge_floor) / resolution + 1;
    int num_points_y = fabsf(edge_right - edge_left) / resolution + 1;

    for (int i = 0; i < num_points_x; ++i)
    {
        pcl::PointXYZ pointX;
        pointX.x = edge_floor + i * resolution;
        pointX.y = edge_left;
        pointX.z = 0.0f;
        cloud.points.push_back(pointX);
        pointX.y = edge_right;
        cloud.points.push_back(pointX);
    }
    for (int j = 0; j < num_points_y; ++j)
    {
        pcl::PointXYZ pointY;
        pointY.x = edge_floor;
        pointY.y = edge_right + j * resolution;
        pointY.z = 0.0f;
        cloud.points.push_back(pointY);
        pointY.x = edge_ceil;
        cloud.points.push_back(pointY);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;

    return cloud;
}

__global__ 
void cloudFilter(pcl::PointXYZ* d_points, int num_points, float dx, float dy, float dyaw, float half_height, float half_width, float floor, float ceil, float left, float right, float max_z, float min_z)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    float x_local_angled = d_points[idx].x * cos(dyaw) - d_points[idx].y * sin(dyaw);
    float y_local_angled = d_points[idx].x * sin(dyaw) + d_points[idx].y * cos(dyaw);

    if ((fabsf(x_local_angled) < half_height && fabsf(y_local_angled) < half_width) || d_points[idx].z < min_z || d_points[idx].z > max_z)
    {
        d_points[idx].x = std::nanf("");
        d_points[idx].y = std::nanf("");
        d_points[idx].z = std::nanf("");
        return;
    }

    d_points[idx].x = x_local_angled + dx;
    d_points[idx].y = y_local_angled + dy;
    d_points[idx].z = 0.0;

    if (d_points[idx].x <= floor || d_points[idx].x >= ceil || d_points[idx].y <= right || d_points[idx].y >= left)
    {
        d_points[idx].x = std::nanf("");
        d_points[idx].y = std::nanf("");
        d_points[idx].z = std::nanf("");
        return;
    }
}

void cloud_cbk(const sensor_msgs::PointCloud2 msg)
{   TIC
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2 cloud_whithout_edge;
    pcl::PointCloud<pcl::PointXYZ> lidarCloud;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(msg, lidarCloud);

    size_t num_points = lidarCloud.points.size();
    size_t num_edpoints = edgeCloud.points.size();
    size_t num_without_edge;
    size_t num_total = num_edpoints + num_points;
    pclCloud.width = num_total;
    pclCloud.height = 1;
    pclCloud.points.resize(num_total);

    pcl::PointXYZ* cloud_data = lidarCloud.points.data();

    try
    {   
        geometry_msgs::TransformStamped transformStamped = buffer.lookupTransform("odom", "pcd_frame", ros::Time(0));

        float dx = transformStamped.transform.translation.x;
        float dy = transformStamped.transform.translation.y;
        float dyaw = transformStamped.transform.translation.z;

        pcl::PointXYZ* d_points;

        // 处理主体点云
        std::thread THREAD_PROCESS_MASTER([&]
        {
            cudaMalloc(&d_points, num_points * sizeof(pcl::PointXYZ));
            cudaMemcpy(d_points, cloud_data, num_points * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice);

            int blockSize = 256;
            int numBlocks = (num_points + blockSize - 1) / blockSize;
            cloudFilter<<<numBlocks, blockSize>>>(d_points, num_points, dx, dy, dyaw, self_half_height, self_half_width, ed_floor, ed_ceil, ed_left, ed_right, passthrough_z_max, passthrough_z_min);

            printf("half_height:%f\n", self_half_height);
            printf("half_width:%f\n", self_half_width);
            printf("floor:%f\n", ed_floor);
            printf("ceil:%f\n", ed_ceil);
            printf("left:%f\n", ed_left);   
            printf("right:%f\n", ed_right);
            printf("max_z:%f\n", passthrough_z_max); 
            printf("min_z:%f\n", passthrough_z_min); 

            cudaMemcpy(pclCloud.points.data(), d_points, num_points * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);
            cudaFree(d_points);
        });

        // 处理边缘点云
        std::thread THRED_PROCESS_EDGE([&]
        {
            for (size_t j = 0; j < edgeCloud.points.size(); j++)
            {
                pclCloud.points[num_points + j] = edgeCloud.points[j];
            }
        });

        THREAD_PROCESS_MASTER.join();
        THRED_PROCESS_EDGE.join();

        // 发布无边缘点云
        std::thread THREAD_PUBLISH_WITHOUT_EDGE([&]
        {
            pcl::PointCloud<pcl::PointXYZ> pclCloudWithoutEdge;
            pclCloudWithoutEdge.points.assign(pclCloud.points.begin(), pclCloud.points.begin() + num_points);

            pclCloudWithoutEdge.points.erase(
            std::remove_if(pclCloudWithoutEdge.points.begin(), pclCloudWithoutEdge.points.end(), [](const pcl::PointXYZ& pt) {
                return std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z);
            }),
            pclCloudWithoutEdge.points.end()
            );

            pclCloudWithoutEdge.width = pclCloudWithoutEdge.points.size();
            pclCloudWithoutEdge.height = 1;
            num_without_edge = pclCloudWithoutEdge.width;

            pcl::toROSMsg(pclCloudWithoutEdge, cloud_whithout_edge);
            cloud_whithout_edge.header.frame_id = "odom";
            pub_without_edge.publish(cloud_whithout_edge);
        });

        // 清除无效点
        pclCloud.points.erase(
            std::remove_if(pclCloud.points.begin(), pclCloud.points.end(), [](const pcl::PointXYZ& pt) {
                return std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z);
            }),
            pclCloud.points.end()
        );

        // 发布完整点云
        pclCloud.width = pclCloud.points.size();
        pcl::toROSMsg(pclCloud, cloud);
        cloud.header.frame_id = "odom";
        pub.publish(cloud);
        TOC
        ROS_INFO("pclCloud size: %lu, raw size: %lu, without edge size: %lu", pclCloud.points.size(), num_total, num_without_edge);

        THREAD_PUBLISH_WITHOUT_EDGE.join();
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("Could not get transform: %s", ex.what());
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cloud_converter");
    ros::NodeHandle nh("~");
    tf2_ros::TransformListener listener(buffer);

    ros::param::get("ed_left", ed_left);
    ros::param::get("ed_right", ed_right);
    ros::param::get("ed_ceil", ed_ceil);
    ros::param::get("ed_floor", ed_floor);
    ros::param::get("ed_resolution", ed_resolution);
    ros::param::get("self_half_height", self_half_height);
    ros::param::get("self_half_width", self_half_width);
    ros::param::get("passthrough_z_max", passthrough_z_max);
    ros::param::get("passthrough_z_min", passthrough_z_min);
    ros::param::get("en_passthrough_z", en_passthrough_z);

    edgeCloud = generateRectanglePointCloud(ed_left, ed_right, ed_floor, ed_ceil, ed_resolution);

    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 10);
    pub_without_edge = nh.advertise<sensor_msgs::PointCloud2>("/cloud_withoutedge", 10);
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloud_cbk);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
