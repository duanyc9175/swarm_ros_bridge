/**
 * @file ros_sub_pub.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file for different ROS message type.
 * 
 * Core Idea: modify the macros about MSG_TYPEx and MSG_CLASSx,
 *  it will generate template functions for different ros message types.
 *  Remember to add the dependent package in find_package() of ../CMakeLists.txt
 * 
 * Note: the sub_cb() and deserialize_pub() are only declared here,
 *  you should define them in you .cpp file according to your need.
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __ROS_SUB_PUB__
#define __ROS_SUB_PUB__
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#define MSG_TYPE1 "nav_msgs/OccupancyGrid"
#define MSG_CLASS1 nav_msgs::OccupancyGrid

#include <geometry_msgs/PoseStamped.h>
#define MSG_TYPE2 "geometry_msgs/PoseStamped"
#define MSG_CLASS2 geometry_msgs::PoseStamped
// 启动指令
#include <swarm_ros_bridge/start.h>
#define MSG_TYPE3 "swarm_ros_bridge/start"
#define MSG_CLASS3 swarm_ros_bridge::start

#include <sensor_msgs/PointCloud2.h>
#define MSG_TYPE4 "sensor_msgs/PointCloud2"
#define MSG_CLASS4 sensor_msgs::PointCloud2

//增加自定义:车辆状态：电量、速度、故障状态
#include <swarm_ros_bridge/ScoutStatus.h>
#define MSG_TYPE5 "swarm_ros_bridge/ScoutStatus"
#define MSG_CLASS5 swarm_ros_bridge::ScoutStatus
 
#include <swarm_ros_bridge/ScoutBmsStatus.h>
#define MSG_TYPE6 "swarm_ros_bridge/ScoutBmsStatus"
#define MSG_CLASS6 swarm_ros_bridge::ScoutBmsStatus

// 增加自定义的ymal文件
#include <swarm_ros_bridge/yamlMsg.h>
#define MSG_TYPE7 "swarm_ros_bridge/yamlMsg"
#define MSG_CLASS7 swarm_ros_bridge::yamlMsg

//发送给小车的运行轨迹
#include <std_msgs/Float64MultiArray.h>
#define MSG_TYPE8 "std_msgs/Float64MultiArray"
#define MSG_CLASS8 std_msgs::Float64MultiArray

// //新增nav_msgs/Path类型的消息
// #include <nav_msgs/Path.h>
// #define MSG_TYPE9 "nav_msgs/Path"
// #define MSG_CLASS9 nav_msgs::Path


//新增CompressedImage类型的消息
#include <sensor_msgs/CompressedImage.h>
#define MSG_TYPE10 "sensor_msgs/CompressedImage"
#define MSG_CLASS10 sensor_msgs::CompressedImage

//新增image类型的消息
#include <sensor_msgs/Image.h>
#define MSG_TYPE11 "sensor_msgs/Image"
#define MSG_CLASS11 sensor_msgs::Image

//新增sensor_msgs/PointCloud类型的消息
#include <sensor_msgs/PointCloud.h>
#define MSG_TYPE12 "sensor_msgs/PointCloud"
#define MSG_CLASS12 sensor_msgs::PointCloud


# define SUB_MAX 70 // max number of subscriber callbacks

template <typename T, int i>
void sub_cb(const T &msg);

template <typename T>
void (*sub_callbacks[])(const T &);

template <typename T>
ros::Subscriber nh_sub(std::string topic_name, ros::NodeHandle nh, int i);

ros::Subscriber topic_subscriber(std::string topic_name, std::string msg_type, ros::NodeHandle nh, int i);

ros::Publisher topic_publisher(std::string topic_name, std::string msg_type, ros::NodeHandle nh);

template<typename T>
void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i);

void deserialize_publish(uint8_t* buffer_ptr, size_t msg_size, std::string msg_type, int i);


template <typename T>
void (*sub_callbacks[])(const T &)=
{
  sub_cb<T,0>, sub_cb<T,1>, sub_cb<T,2>, sub_cb<T,3>, sub_cb<T,4>,
  sub_cb<T,5>, sub_cb<T,6>, sub_cb<T,7>, sub_cb<T,8>, sub_cb<T,9>,
  sub_cb<T,10>, sub_cb<T,11>, sub_cb<T,12>, sub_cb<T,13>, sub_cb<T,14>,
  sub_cb<T,15>, sub_cb<T,16>, sub_cb<T,17>, sub_cb<T,18>, sub_cb<T,19>,
  sub_cb<T,20>, sub_cb<T,21>, sub_cb<T,22>, sub_cb<T,23>, sub_cb<T,24>,
  sub_cb<T,25>, sub_cb<T,26>, sub_cb<T,27>, sub_cb<T,28>, sub_cb<T,29>,
  sub_cb<T,30>, sub_cb<T,31>, sub_cb<T,32>, sub_cb<T,33>, sub_cb<T,34>,
  sub_cb<T,35>, sub_cb<T,36>, sub_cb<T,37>, sub_cb<T,38>, sub_cb<T,39>,
  sub_cb<T,40>, sub_cb<T,41>, sub_cb<T,42>, sub_cb<T,43>, sub_cb<T,44>,
  sub_cb<T,45>, sub_cb<T,46>, sub_cb<T,47>, sub_cb<T,48>, sub_cb<T,49>,
  sub_cb<T,50>, sub_cb<T,51>, sub_cb<T,52>, sub_cb<T,53>, sub_cb<T,54>,
  sub_cb<T,55>, sub_cb<T,56>, sub_cb<T,57>, sub_cb<T,58>, sub_cb<T,59>,
  sub_cb<T,60>, sub_cb<T,61>, sub_cb<T,62>, sub_cb<T,63>, sub_cb<T,64>,
  sub_cb<T,65>, sub_cb<T,66>, sub_cb<T,67>, sub_cb<T,68>, sub_cb<T,69>,
};

template <typename T>
ros::Subscriber nh_sub(std::string topic_name, ros::NodeHandle nh, int i)
{
  return nh.subscribe(topic_name, 10, sub_callbacks<T>[i], ros::TransportHints().tcpNoDelay());
}

ros::Subscriber topic_subscriber(std::string topic_name, std::string msg_type, ros::NodeHandle nh, int i)
{
  #ifdef MSG_TYPE1
    if (msg_type == MSG_TYPE1)
        return nh_sub<MSG_CLASS1>(topic_name, nh, i);
  #endif
  #ifdef MSG_TYPE2
    if (msg_type == MSG_TYPE2)
        return nh_sub<MSG_CLASS2>(topic_name, nh, i);
  #endif
  #ifdef MSG_TYPE3
    if (msg_type == MSG_TYPE3)
        return nh_sub<MSG_CLASS3>(topic_name, nh, i);
  #endif
  #ifdef MSG_TYPE4
    if (msg_type == MSG_TYPE4)
        return nh_sub<MSG_CLASS4>(topic_name, nh, i);
  #endif  
  #ifdef MSG_TYPE5
    if (msg_type == MSG_TYPE5)
        return nh_sub<MSG_CLASS5>(topic_name, nh, i);
  #endif  
  #ifdef MSG_TYPE6
    if (msg_type == MSG_TYPE6)
        return nh_sub<MSG_CLASS6>(topic_name, nh, i);
  #endif  
  #ifdef MSG_TYPE7
    if (msg_type == MSG_TYPE7)
        return nh_sub<MSG_CLASS7>(topic_name, nh, i);
  #endif  
  #ifdef MSG_TYPE8
    if (msg_type == MSG_TYPE8)
        return nh_sub<MSG_CLASS8>(topic_name, nh, i);
  #endif  
  #ifdef MSG_TYPE9
    if (msg_type == MSG_TYPE9)
        return nh_sub<MSG_CLASS9>(topic_name, nh, i);
  #endif  
  #ifdef MSG_TYPE10
    if (msg_type == MSG_TYPE10)
        return nh_sub<MSG_CLASS10>(topic_name, nh, i);
  #endif
  #ifdef MSG_TYPE11
    if (msg_type == MSG_TYPE11)
        return nh_sub<MSG_CLASS11>(topic_name, nh, i);
  #endif
  #ifdef MSG_TYPE12
    if (msg_type == MSG_TYPE12)
        return nh_sub<MSG_CLASS12>(topic_name, nh, i);
  #endif  
    ROS_FATAL("Invalid ROS msg_type \"%s\" in configuration!", msg_type.c_str());
    exit(1);
}

ros::Publisher topic_publisher(std::string topic_name, std::string msg_type, ros::NodeHandle nh)
{
  #ifdef MSG_TYPE1
    if (msg_type == MSG_TYPE1)
        return nh.advertise<MSG_CLASS1>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE2
    if (msg_type == MSG_TYPE2)
        return nh.advertise<MSG_CLASS2>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE3
    if (msg_type == MSG_TYPE3)
        return nh.advertise<MSG_CLASS3>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE4
    if (msg_type == MSG_TYPE4)
        return nh.advertise<MSG_CLASS4>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE5
    if (msg_type == MSG_TYPE5)
        return nh.advertise<MSG_CLASS5>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE6
    if (msg_type == MSG_TYPE6)
        return nh.advertise<MSG_CLASS6>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE7
    if (msg_type == MSG_TYPE7)
        return nh.advertise<MSG_CLASS7>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE8
    if (msg_type == MSG_TYPE8)
        return nh.advertise<MSG_CLASS8>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE9
    if (msg_type == MSG_TYPE9)
        return nh.advertise<MSG_CLASS9>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE10
    if (msg_type == MSG_TYPE10)
        return nh.advertise<MSG_CLASS10>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE11
    if (msg_type == MSG_TYPE11)
        return nh.advertise<MSG_CLASS11>(topic_name, 10);
  #endif
  #ifdef MSG_TYPE12
    if (msg_type == MSG_TYPE12)
        return nh.advertise<MSG_CLASS12>(topic_name, 10);
  #endif
    ROS_FATAL("Invalid ROS msg_type \"%s\" in configuration!", msg_type.c_str());
    exit(1);
}

void deserialize_publish(uint8_t* buffer_ptr, size_t msg_size, std::string msg_type, int i)
{
  #ifdef MSG_TYPE1
    if (msg_type == MSG_TYPE1)
        return deserialize_pub<MSG_CLASS1>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE2
    if (msg_type == MSG_TYPE2)
        return deserialize_pub<MSG_CLASS2>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE3
    if (msg_type == MSG_TYPE3)
        return deserialize_pub<MSG_CLASS3>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE4
    if (msg_type == MSG_TYPE4)
        return deserialize_pub<MSG_CLASS4>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE5
    if (msg_type == MSG_TYPE5)
        return deserialize_pub<MSG_CLASS5>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE6
    if (msg_type == MSG_TYPE6)
        return deserialize_pub<MSG_CLASS6>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE7
    if (msg_type == MSG_TYPE7)
        return deserialize_pub<MSG_CLASS7>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE8
    if (msg_type == MSG_TYPE8)
        return deserialize_pub<MSG_CLASS8>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE9
    if (msg_type == MSG_TYPE9)
        return deserialize_pub<MSG_CLASS9>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE10
    if (msg_type == MSG_TYPE10)
        return deserialize_pub<MSG_CLASS10>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE11
    if (msg_type == MSG_TYPE11)
        return deserialize_pub<MSG_CLASS11>(buffer_ptr, msg_size, i);
  #endif
  #ifdef MSG_TYPE12
    if (msg_type == MSG_TYPE12)
        return deserialize_pub<MSG_CLASS12>(buffer_ptr, msg_size, i);
  #endif
    ROS_FATAL("Invalid ROS msg_type \"%s\" in configuration!", msg_type.c_str());
    exit(1);
}

#endif