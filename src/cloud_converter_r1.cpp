/*
 * @Author: hejia 2736463842@qq.com
 * @Date: 2025-01-19 22:39:05
 * @LastEditors: hejia 2736463842@qq.com
 * @LastEditTime: 2025-02-02 22:09:34
 * @FilePath: /ego-planner-swarm/src/World_Cloud_Converter/src/cloud_converter.cpp
 * @Description: Convert PointCloud coordinates using TF (left、right、back —— 1、2、3)
 */

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

 #include "tictoc.hpp"
 
 #define quarter_pi 0.785398163397f
 #define pi 3.141592654f
 
 float ed_left, ed_right, ed_ceil, ed_floor, ed_resolution;
 
 Eigen::Matrix2d rotation_1;         // left
 Eigen::Vector2d transmission_1;
 Eigen::Matrix2d rotation_2;         // right
 Eigen::Vector2d transmission_2;
 
 float yaw_1, yaw_2;
 float half_height, half_width;
 
 std::string cloud_topic_1;
 std::string cloud_topic_2;
 
 tf2_ros::Buffer buffer;
 ros::Publisher pub;
 ros::Publisher pub_without_edge;
 pcl::PointCloud<pcl::PointXYZ> edgeCloud;
 
 using namespace message_filters;
 
 /**
  * @brief Generate a point cloud with a rectangular shape
  *
  * @param edge_left   The left edge of the rectangle
  * @param edge_right  The right edge of the rectangle
  * @param edge_floor  The floor edge of the rectangle
  * @param edge_ceil   The ceil edge of the rectangle
  * @param resolution  The distance between two points
  *
  * @return A point cloud with a rectangular shape
  */
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
 
 /**
  * @brief Callback function to process and transform multiple point clouds.
  * 
  * This function receives three input point cloud messages, transforms their coordinates using
  * transformation data, and merges them into a single point cloud with additional edge points.
  * The resulting point cloud is then published.
  * 
  * @param msg_1 First input point cloud message containing the front points.
  * @param msg_2 Second input point cloud message containing the right points.
  * 
  * The transformation process involves applying a rotation and translation based on the
  * transformation data obtained from the TF buffer. Each point cloud is processed in parallel 
  * threads, taking into consideration the defined boundaries and ranges, to ensure valid points 
  * are included in the final output.
  */
 void cloud_cbk(const sensor_msgs::PointCloud::ConstPtr &msg_1, const sensor_msgs::PointCloud::ConstPtr &msg_2)
 {
    TIC
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud2 cloud_whithout_edge;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;

    size_t num_fpoints = msg_1->points.size();
    size_t num_spoints = msg_2->points.size();
    size_t num_without_edge = num_fpoints + num_spoints;
    size_t num_edpoints = edgeCloud.points.size();
    size_t num_total = num_fpoints + num_spoints + num_edpoints;
    pclCloud.width = num_total;
    pclCloud.height = 1;
    pclCloud.points.resize(num_total);

    try
    {
        geometry_msgs::TransformStamped transformStamped = buffer.lookupTransform("odom", "pcd_frame", ros::Time(0));

        float dx = transformStamped.transform.translation.x;
        float dy = transformStamped.transform.translation.y;
        float dyaw = transformStamped.transform.translation.z;

        Eigen::Matrix2d rotation;
        rotation << cos(dyaw), -sin(dyaw),
                    sin(dyaw),  cos(dyaw);
        Eigen::Vector2d transmission(dx, dy);

        std::thread THREAD_PROCESS_FIRST([&]
        {
            for (size_t i = 0; i < num_fpoints; i++)
            {
                Eigen::Vector2d point(msg_1->points[i].x, msg_1->points[i].y);
                Eigen::Vector2d point_rbt(rotation_1 * point + transmission_1);

                if (fabs(point_rbt(0)) <= half_height && fabs(point_rbt(1)) <= half_width)
                {
                    pclCloud.points[i].x = std::nanf("");
                    pclCloud.points[i].y = std::nanf("");
                    pclCloud.points[i].z = std::nanf("");
                    continue;
                }

                Eigen::Vector2d point_world = rotation * point_rbt + transmission;

                if (point_world(0) <= ed_floor || point_world(0) >= ed_ceil || point_world(1) <= ed_right || point_world(1) >= ed_left)
                {
                    pclCloud.points[i].x = std::nanf("");
                    pclCloud.points[i].y = std::nanf("");
                    pclCloud.points[i].z = std::nanf("");
                    continue;
                }               

                pclCloud.points[i].x = point_world(0);
                pclCloud.points[i].y = point_world(1);
                pclCloud.points[i].z = 0.0;
            } 
        });

        std::thread THREAD_PROCESS_SECOND([&]
        {
            for (size_t i = 0; i < num_spoints; i++)
            {
                size_t index = num_fpoints + i;
                Eigen::Vector2d point(msg_2->points[index].x, msg_2->points[index].y);
                Eigen::Vector2d point_rbt(rotation_2 * point + transmission_2);

                if (fabs(point_rbt(0)) <= half_height && fabs(point_rbt(1)) <= half_width)
                {
                    pclCloud.points[index].x = std::nanf("");
                    pclCloud.points[index].y = std::nanf("");
                    pclCloud.points[index].z = std::nanf("");
                    continue;
                }

                Eigen::Vector2d point_world = rotation * point_rbt + transmission;

                if (point_world(0) <= ed_floor || point_world(0) >= ed_ceil || point_world(1) <= ed_right || point_world(1) >= ed_left)
                {
                    pclCloud.points[index].x = std::nanf("");
                    pclCloud.points[index].y = std::nanf("");
                    pclCloud.points[index].z = std::nanf("");
                    continue;
                }               

                pclCloud.points[index].x = point_world(0);
                pclCloud.points[index].y = point_world(1);
                pclCloud.points[index].z = 0.0;
            } 
        });

        std::thread THRED_PROCESS_EDGE([&]
        {
            for (size_t j = 0; j < edgeCloud.points.size(); j++)
            {
                pclCloud.points[num_fpoints + num_spoints + j] = edgeCloud.points[j];
            }
        });

        THREAD_PROCESS_FIRST.join();
        THREAD_PROCESS_SECOND.join();
        THRED_PROCESS_EDGE.join();

        // 发布无边缘点云
        std::thread THREAD_PUBLISH_WITHOUT_EDGE([&]
        {
            pcl::PointCloud<pcl::PointXYZ> pclCloudWithoutEdge;
            pclCloudWithoutEdge.points.assign(pclCloud.points.begin(), pclCloud.points.begin() + num_without_edge);

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

        pclCloud.width = pclCloud.points.size();
        pcl::toROSMsg(pclCloud, cloud);
        cloud.header.frame_id = "odom";
        pub.publish(cloud);
        TOC
       
        THREAD_PUBLISH_WITHOUT_EDGE.join();
        ROS_INFO("pclCloud size: %lu, raw size: %lu, without edge size: %lu", pclCloud.points.size(), num_total, num_without_edge);
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("Could not get transform: %s", ex.what());
    }
}
 
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cloud_converter");
    ros::NodeHandle nh;

    ros::param::get("ed_left", ed_left);                    // 左侧边界
    ros::param::get("ed_right", ed_right);                  // 右侧边界
    ros::param::get("ed_ceil", ed_ceil);                    // 上侧边界
    ros::param::get("ed_floor", ed_floor);                  // 下侧边界
    ros::param::get("ed_resolution", ed_resolution);        // 边界点云分辨率
    ros::param::get("yaw_1", yaw_1);                        // 1号雷达坐标系与车体坐标系的偏差
    ros::param::get("yaw_2", yaw_2);                        // 2号雷达坐标系与车体坐标系的偏差
    ros::param::get("dist1_x", transmission_1(0));          // 1号雷达在车体坐标系下的坐标
    ros::param::get("dist1_y", transmission_1(1));
    ros::param::get("dist2_x", transmission_2(0));          // 2号雷达在车体坐标系下的坐标
    ros::param::get("dist2_y", transmission_2(1));
    ros::param::get("half_height", half_height);            // 车体高度的一半
    ros::param::get("half_width", half_width);              // 车体宽度的一半
    ros::param::get("cloud_topic_1", cloud_topic_1);        // 1号雷达点云话题
    ros::param::get("cloud_topic_2", cloud_topic_2);        // 2号雷达点云话题
    
    rotation_1 << cos(yaw_1), -sin(yaw_1),                  // 旋转矩阵 1
                sin(yaw_1),  cos(yaw_1);
    rotation_2 << cos(yaw_2), -sin(yaw_2),                  // 旋转矩阵 2
                sin(yaw_2),  cos(yaw_2);

    tf2_ros::TransformListener listener(buffer);
    edgeCloud = generateRectanglePointCloud(ed_left, ed_right, ed_floor, ed_ceil, ed_resolution);

    // Subscriber and Publisher
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_1(nh, cloud_topic_1, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_2(nh, cloud_topic_2, 1);

    // Time Sync
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud, sensor_msgs::PointCloud, sensor_msgs::PointCloud> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2);
    sync.registerCallback(boost::bind(&cloud_cbk, _1, _2));

    // Asynchronous Callback
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
