/*
 * @FilePath     : /src/World_Cloud_Converter/src/cloud_converter_cuda_r1.cu
 * @Description  :  
 * @Author       : hejia 2736463842@qq.com
 * @Version      : 0.0.1
 * @LastEditors  : hejia 2736463842@qq.com
 * @LastEditTime : 2025-03-09 16:14:06
 * Copyright    : G AUTOMOBILE RESEARCH INSTITUTE CO.,LTD Copyright (c) 2025.
 */

#include "parameters.hpp"
#include "tictoc.hpp"

/**
 * @brief Generate a point cloud with a rectangular shape
 * @param edge_left   The left edge of the rectangle
 * @param edge_right  The right edge of the rectangle
 * @param edge_floor  The floor edge of the rectangle
 * @param edge_ceil   The ceil edge of the rectangle
 * @param resolution  The distance between two points
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
 * @brief Init the parameters
 */
void initParameters(void)
{
    // Get Parameters
    ros::param::get("ed_left", ed_left);                    // 左侧边界
    ros::param::get("ed_right", ed_right);                  // 右侧边界
    ros::param::get("ed_ceil", ed_ceil);                    // 上侧边界
    ros::param::get("ed_floor", ed_floor);                  // 下侧边界
    ros::param::get("ed_resolution", ed_resolution);        // 边界点云分辨率
    ros::param::get("passthrough_z_max", passthrough_z_max);// z轴直通滤波
    ros::param::get("passthrough_z_min", passthrough_z_min);// z轴直通滤波
    ros::param::get("half_height", half_height);            // 车体高度的一半
    ros::param::get("half_width", half_width);              // 车体宽度的一半
    ros::param::get("cloud_topic_1", cloud_topic_1);        // 1号雷达点云话题
    ros::param::get("cloud_topic_2", cloud_topic_2);        // 2号雷达点云话题

    edgeCloud = generateRectanglePointCloud(ed_left, ed_right, ed_floor, ed_ceil, ed_resolution);
}

__global__ 
void cloudFilter(livox_ros_driver::CustomPoint* points, size_t num_points, float left, float right, float floor, float ceil, float z_max, float z_min, float dyaw, float dx, float dy, float self_half_height, float self_half_width)
{
    size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    float x_local, y_local, z_local;

    x_local = points[idx].x;
    y_local = points[idx].y;
    z_local = points[idx].z;

    if(fabsf(x_local) < self_half_height && fabsf(y_local) < self_half_width || z_local < z_min || z_local > z_max)
    {
        points[idx].x = std::nanf("");
        points[idx].y = std::nanf("");
        points[idx].z = std::nanf("");
        return;
    }

    points[idx].x = x_local * cos(dyaw) - y_local * sin(dyaw) + dx;
    points[idx].y = x_local * sin(dyaw) + y_local * cos(dyaw) + dy;
    points[idx].z = 0.0f;

    if(points[idx].x <= floor || points[idx].x >= ceil || points[idx].y <= right || points[idx].y >= left)
    {
        points[idx].x = std::nanf("");
        points[idx].y = std::nanf("");
        points[idx].z = std::nanf("");
        return;
    }
}
    
void livox_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_1, const livox_ros_driver::CustomMsg::ConstPtr &msg_2)
{
    sensor_msgs::PointCloud2 cloud;                 // 转换后的雷达点云
    sensor_msgs::PointCloud2 cloud_whithout_edge;   // 去除边界后的雷达点云
    pcl::PointCloud<pcl::PointXYZ> pclCloud;        

    size_t num_points_1 = msg_1->point_num;
    size_t num_points_2 = msg_2->point_num;
    size_t num_edpoints = edgeCloud.points.size();
    size_t num_without_edge = num_points_1 + num_points_2;
    size_t num_total = num_edpoints + num_without_edge;
    pclCloud.width = num_total;
    pclCloud.height = 1;
    pclCloud.points.resize(num_total);

    std::vector<livox_ros_driver::CustomPoint> points = msg_1->points;
    points.insert(points.end(), msg_2->points.begin(), msg_2->points.end());

    try{
        geometry_msgs::TransformStamped transformStamped = buffer.lookupTransform("odom", "pcd_frame", ros::Time(0));

        float dx = transformStamped.transform.translation.x;
        float dy = transformStamped.transform.translation.y;
        float dyaw = transformStamped.transform.translation.z;

        // 处理主体点云
        std::thread THREAD_PROCESS_MASTER([&]
        {
            livox_ros_driver::CustomPoint* d_points;
            
            cudaMalloc(&d_points, num_without_edge * sizeof(livox_ros_driver::CustomPoint));
            cudaMemcpy(d_points, points.data(), num_without_edge * sizeof(livox_ros_driver::CustomPoint), cudaMemcpyHostToDevice);
    
            int blockSize = 256;
            int numBlocks = (num_without_edge + blockSize - 1) / blockSize;
            cloudFilter<<<numBlocks, blockSize>>>(d_points, num_without_edge, ed_left, ed_right, ed_floor, ed_ceil, passthrough_z_max, passthrough_z_min, dyaw, dx, dy, half_height, half_width);
    
            printf("half_height:%f\n", half_height);
            printf("half_width:%f\n", half_width);
            printf("floor:%f\n", ed_floor);
            printf("ceil:%f\n", ed_ceil);
            printf("left:%f\n", ed_left);   
            printf("right:%f\n", ed_right);
            printf("max_z:%f\n", passthrough_z_max); 
            printf("min_z:%f\n", passthrough_z_min); 
    
            cudaMemcpy(pclCloud.points.data(), d_points, num_without_edge * sizeof(livox_ros_driver::CustomPoint), cudaMemcpyDeviceToHost);
            cudaFree(d_points);
        });

        // 处理边缘点云
        std::thread THRED_PROCESS_EDGE([&]
        {
            for (size_t j = 0; j < edgeCloud.points.size(); j++)
            {
                pclCloud.points[num_without_edge + j] = edgeCloud.points[j];
            }
        });

        THREAD_PROCESS_MASTER.join();
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
    // Init
    ros::init(argc, argv, "cloud_converter");
    ros::NodeHandle nh("~");
    tf2_ros::TransformListener listener(buffer);  
    initParameters();

    // Sub and Pub
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 10);
    pub_without_edge = nh.advertise<sensor_msgs::PointCloud2>("/cloud_withoutedge", 10);
    message_filters::Subscriber<livox_ros_driver::CustomMsg> sub_1(nh, cloud_topic_1, 1);
    message_filters::Subscriber<livox_ros_driver::CustomMsg> sub_2(nh, cloud_topic_2, 1);
    
    // Time Sync
    typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver::CustomMsg, livox_ros_driver::CustomMsg> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2);
    sync.registerCallback(boost::bind(&livox_cbk, _1, _2));

    // Asynchronous Callback
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}