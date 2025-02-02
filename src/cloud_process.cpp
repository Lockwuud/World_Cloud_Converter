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

#define self_range 0.35
#define ed_left 4.62524
#define ed_right -0.32476
#define ed_ceil 7.63428
#define ed_floor -0.36572
#define ed_resolution 0.2
#define quarter_pi 0.785398163397f
#define three_quarters_pi 2.356194490191f
#define half_length 257.525f

#define thread_nums 4

using namespace message_filters;

tf2_ros::Buffer buffer;
ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ> edgeCloud;
std::vector<sensor_msgs::PointCloud> queueCloud_f;
std::vector<sensor_msgs::PointCloud> queueCloud_r;

class ThreadPool
{
private:
    // 线程池中的工作线程函数
    void workerLoop();

    std::vector<std::thread> workers;        // 线程池中的所有线程
    std::queue<std::function<void()>> tasks; // 任务队列，存储待处理的任务
    std::mutex queueMutex;                   // 互斥锁，用于保护任务队列
    std::condition_variable condVar;         // 条件变量，用于通知线程执行任务
    std::atomic<bool> stop;                  // 原子变量，用于控制线程池的停止

public:
    ThreadPool(size_t numThreads);

    ~ThreadPool();

    void enqueue(std::function<void()> task);
};

/**
 * @description: 构造函数，初始化线程池
 * @param {size_t} numThreads
 * @return {*}
 */
ThreadPool::ThreadPool(size_t numThreads) : stop(false) {
    for (size_t i = 0; i < numThreads; ++i) {
        workers.push_back(std::thread([this]() { workerLoop(); }));
    }
}

/**
 * @description: 析构函数，停止线程池中的所有线程
 * @return {*}
 */
ThreadPool::~ThreadPool() {
    stop = true;         
    condVar.notify_all();  
    for (std::thread &worker : workers) {
        worker.join();     
    }
}

/**
 * @description: 向线程池队列中添加一个任务
 * @param {function<void()>} task
 * @return {*}
 */
void ThreadPool::enqueue(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(queueMutex); 
        tasks.push(task);  
    }
    condVar.notify_one();
}

/**
 * @description: 线程池工作循环
 * @return {*}
 */
void ThreadPool::workerLoop() {
    while (!stop) 
    { 
        std::function<void()> task;  
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            condVar.wait(lock, [this]() { return stop || !tasks.empty(); });  // 等待任务或停止信号

            if (stop && tasks.empty()) {
                return;
            }

            task = tasks.front();
            tasks.pop(); 
        }

        task();
    }
}

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

void cloud_front_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    queueCloud_f.push_bcak(msg);
}

void cloud_rear_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    queueCloud_r.push_back(msg);
}

void cloud_cbk(const sensor_msgs::PointCloud msg_f, const sensor_msgs::PointCloud msg_r)
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

        std::thread THREAD_PROCESS_FRONT([&]
                                         {
            float dyaw_ = dyaw - quarter_pi;
            for (size_t i = 0; i < num_fpoints; i++)
            {
                float x_local_angled = msg_f->points[i].x * cos(dyaw_) - msg_f->points[i].y * sin(dyaw_);
                float y_local_angled = msg_f->points[i].x * sin(dyaw_) + msg_f->points[i].y * cos(dyaw_);

                if (x_local_angled < 0 && y_local_angled > 0)
                {
                    pclCloud.points[i].x = std::nanf("");
                    pclCloud.points[i].y = std::nanf("");
                    pclCloud.points[i].z = std::nanf("");
                    continue;
                }

                pclCloud.points[i].x = x_local_angled + half_length + dx;
                pclCloud.points[i].y = y_local_angled - half_length + dy;
                pclCloud.points[i].z = 0.0;

                if (pclCloud.points[i].x <= ed_floor || pclCloud.points[i].x >= ed_ceil || pclCloud.points[i].y <= ed_right || pclCloud.points[i].y >= ed_left)
                {
                    pclCloud.points[i].x = std::nanf("");
                    pclCloud.points[i].y = std::nanf("");
                    pclCloud.points[i].z = std::nanf("");
                    continue;
                }
            } });

        std::thread THREAD_PROCESS_REAR([&]
                                        {
            float dyaw_ = dyaw + three_quarters_pi;
            for (size_t i = 0; i < num_rpoints; i++)
            {
                size_t j = i + num_fpoints;
                float x_local_angled = msg_r->points[i].x * cos(dyaw_) - msg_r->points[i].y * sin(dyaw_);
                float y_local_angled = msg_r->points[i].x * sin(dyaw_) + msg_r->points[i].y * cos(dyaw_);

                if (x_local_angled > 0 && y_local_angled < 0)
                {
                    pclCloud.points[j].x = std::nanf("");
                    pclCloud.points[j].y = std::nanf("");
                    pclCloud.points[j].z = std::nanf("");
                    continue;
                }

                pclCloud.points[j].x = x_local_angled - half_length + dx;
                pclCloud.points[j].y = y_local_angled + half_length + dy;
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

        THREAD_PROCESS_FRONT.join();
        THREAD_PROCESS_REAR.join();
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
    ThreadPool threadPool(4);
    ros::init(argc, argv, "cloud_converter");
    ros::NodeHandle nh;
    tf2_ros::TransformListener listener(buffer);
    edgeCloud = generateRectanglePointCloud(ed_left, ed_right, ed_floor, ed_ceil, ed_resolution);

    // Subscriber and Publisher
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 10);
    ros::Subscriber sub_front = nh.subscribe("/cloud_front", 1, cloud_front_cbk);
    ros::Subscriber sub_rear = nh.subscribe("/cloud_rear", 1, cloud_rear_cbk);
 
    // Queue Sync
    std::thread THREAD_CLOUD_SYNC([&]{
        if(!queueCloud_f.empty() && !queueCloud_r.empty())
        {
            if(queueCloud_r.size() < 2){
                sensor_msgs::PointCloud2 msg_f = queueCloud_f.front();
                sensor_msgs::PointCloud2 msg_r = queueCloud_r.front();
            }
            else{
                while(queueCloud_r.size() >= 2)
                {
                    sensor_msgs::PointCloud2 msg_r_next = queueCloud_r[queueCloud_r.begin
                ()+1];
                }
            }
            

        }
    });

    // MultiThreaded Spinner
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    THREAD_CLOUD_SYNC.join();
    return 0;
}
