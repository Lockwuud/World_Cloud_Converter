/**
 * @FilePath     : /src/World_Cloud_Converter/src/cloud_converter_r1.cpp
 * @Description  :  
 * @Author       : hejia 2736463842@qq.com
 * @Version      : 0.0.1
 * @LastEditors  : hejia 2736463842@qq.com
 * @LastEditTime : 2025-02-20 16:54:05
 * @Copyright    : G AUTOMOBILE RESEARCH INSTITUTE CO.,LTD Copyright (c) 2025.
**/

#include <thread>

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

#define quarter_pi 0.785398163397f

using namespace message_filters;

float ed_left, ed_right, ed_ceil, ed_floor, ed_resolution, self_h, self_w, range;

tf2_ros::Buffer buffer;                     // tf转换
ros::Publisher pub;                         // 发布处理后的点云
pcl::PointCloud<pcl::PointXYZ> edgeCloud;   // 边界点云


pcl::PointCloud<pcl::PointXYZ> generateRectanglePointCloud(float edge_left, float edge_right, float edge_floor, float edge_ceil, float resolution)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // 计算矩形的边界点数
    int num_points_x = fabsf(edge_ceil - edge_floor) / resolution + 1;
    int num_points_y = fabsf(edge_right - edge_left) / resolution + 1;

    // 生成矩形点云
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

    // 设置点云的宽高
    cloud.width = cloud.points.size();
    cloud.height = 1;

    return cloud;
}

void cloud_cbk(const sensor_msgs::PointCloud::ConstPtr &msg_f, const sensor_msgs::PointCloud::ConstPtr &msg_r)
{
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    size_t num_fpoints = msg_f->points.size();
    size_t num_rpoints = msg_r->points.size();
    size_t num_edpoints = edgeCloud.points.size();
    size_t num_total = num_fpoints + num_rpoints + num_edpoints;
    pclCloud.width = num_total;
    pclCloud.height = 1;
    pclCloud.points.resize(num_total);

    try
    {
        geometry_msgs::TransformStamped transformStamped = buffer.lookupTransform("odom", "pcd_frame", ros::Time(0));

        float dx = transformStamped.transform.translation.x;
        float dy = transformStamped.transform.translation.y;
        float dyaw = transformStamped.transform.translation.z;

        std::thread THREAD_PROCESS_RIGHT([&]
                                         {
            float dyaw_ = dyaw - quarter_pi;
            for (size_t i = 0; i < num_fpoints; i++)
            {
                float x_local_angled = msg_f->points[i].x * cos(dyaw_) - msg_f->points[i].y * sin(dyaw_);
                float y_local_angled = msg_f->points[i].x * sin(dyaw_) + msg_f->points[i].y * cos(dyaw_);

                pclCloud.points[i].x = x_local_angled + self_h;
                pclCloud.points[i].y = y_local_angled - self_w;

                if (fabs(pclCloud.points[i].x) <= range && fabs(pclCloud.points[i].y) <= range)
                {
                    pclCloud.points[i].x = std::nanf("");
                    pclCloud.points[i].y = std::nanf("");
                    pclCloud.points[i].z = std::nanf("");
                    continue;
                }

                pclCloud.points[i].x += dx;
                pclCloud.points[i].y += dy;
                pclCloud.points[i].z = 0.0;

                if (pclCloud.points[i].x <= ed_floor || pclCloud.points[i].x >= ed_ceil || pclCloud.points[i].y <= ed_right || pclCloud.points[i].y >= ed_left)
                {
                    pclCloud.points[i].x = std::nanf("");
                    pclCloud.points[i].y = std::nanf("");
                    pclCloud.points[i].z = std::nanf("");
                    continue;
                }               
            } });

        std::thread THREAD_PROCESS_LEFT([&]
                                        {
            float dyaw_ = dyaw + quarter_pi;
            for (size_t i = 0; i < num_rpoints; i++)
            {
                size_t j = i + num_fpoints;
                float x_local_angled = msg_r->points[i].x * cos(dyaw_) - msg_r->points[i].y * sin(dyaw_);
                float y_local_angled = msg_r->points[i].x * sin(dyaw_) + msg_r->points[i].y * cos(dyaw_);

                if(x_local_angled >= -0.3 && y_local_angled <= 0.2)
                {
                    pclCloud.points[j].x = std::nanf("");
                    pclCloud.points[j].y = std::nanf("");
                    pclCloud.points[j].z = std::nanf("");
                    continue;
                }

                pclCloud.points[j].x = x_local_angled + self_h;
                pclCloud.points[j].y = y_local_angled + self_w;

                if (fabsf(pclCloud.points[i].x) <= range && fabs(pclCloud.points[i].y) <= range)
                {
                    pclCloud.points[j].x = std::nanf("");
                    pclCloud.points[j].y = std::nanf("");
                    pclCloud.points[j].z = std::nanf("");
                    continue;
                }

                pclCloud.points[j].x += dx;
                pclCloud.points[j].y += dy;
                pclCloud.points[j].z = 0.0;

                if (pclCloud.points[j].x <= ed_floor || pclCloud.points[j].x >= ed_ceil || pclCloud.points[j].y <= ed_right || pclCloud.points[j].y >= ed_left)
                {
                    pclCloud.points[j].x = std::nanf("");
                    pclCloud.points[j].y = std::nanf("");
                    pclCloud.points[j].z = std::nanf("");
                    continue;
                }
            } });

        std::thread THRED_PROCESS_EDGE([&]
            {
                for (size_t j = 0; j < edgeCloud.points.size(); j++)
                {
                    pclCloud.points[num_fpoints + num_rpoints + j] = edgeCloud.points[j];
                }
            });

        THREAD_PROCESS_RIGHT.join();
        THREAD_PROCESS_LEFT.join();
        THRED_PROCESS_EDGE.join();

        pcl::toROSMsg(pclCloud, cloud);
        cloud.header.frame_id = "odom";
        pub.publish(cloud);
        ROS_INFO("pclCloud size: %lu", pclCloud.points.size());
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("Could not get transform: %s", ex.what());
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cloud_converter");
    ros::NodeHandle nh;
    ros::param::get("ed_left", ed_left);
    ros::param::get("ed_right", ed_right);
    ros::param::get("ed_ceil", ed_ceil);
    ros::param::get("ed_floor", ed_floor);
    ros::param::get("ed_resolution", ed_resolution);
    ros::param::get("self_half_height", self_h);
    ros::param::get("self_half_width", self_w);
    ros::param::get("range", range);

    tf2_ros::TransformListener listener(buffer);
    edgeCloud = generateRectanglePointCloud(ed_left, ed_right, ed_floor, ed_ceil, ed_resolution);

    // Subscriber and Publisher
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud> right_sub(nh, "/cloud_right", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud> left_sub(nh, "/cloud_left", 1);

    // Time Sync
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud, sensor_msgs::PointCloud> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), right_sub, left_sub); // queue size=10
    sync.registerCallback(boost::bind(&cloud_cbk, _1, _2));

    // Asynchronous Callback
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
