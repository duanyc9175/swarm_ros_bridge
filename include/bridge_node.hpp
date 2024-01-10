/**
 * @file bridge_node.hpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Header file of bridge_node.cpp
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#ifndef __BRIDGE_NODE__
#define __BRIDGE_NODE__
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <unistd.h>
#include <string>
#include <zmqpp/zmqpp.hpp>
/*
zmqpp is the c++ wrapper around ZeroMQ
Intall zmqpp first:
    sudo apt install libzmqpp-dev
zmqpp reference link:
    https://zeromq.github.io/zmqpp/namespacezmqpp.html
*/
#include "ros_sub_pub.hpp"

//结构体，TopicInfo
struct TopicInfo
{
  std::string name;
  std::string type;
  int max_freq;
  std::string ip;
  int port;
};

//********************* Parse configuration file **************************
std::string ns; // namespace of this node
// 存储IP、发送主题和接收主题的XML-RPC值
XmlRpc::XmlRpcValue ip_xml;
XmlRpc::XmlRpcValue send_topics_xml;
XmlRpc::XmlRpcValue recv_topics_xml;
// 发送主题和接收主题的数量
int len_send; // length(number) of send topics
int len_recv; // length(number) of receive topics
// 一个映射，用于存储主机名和IP地址的映射关系
std::map<std::string, std::string> ip_map; // map host name and IP

std::vector<TopicInfo> sendTopics; // send topics info struct vector
std::vector<TopicInfo> recvTopics; // receive topics info struct vector

// ********************* zmq socket initialize ***************************
zmqpp::context_t context;
std::vector<std::unique_ptr<zmqpp::socket>> senders;   //index senders
std::vector<std::unique_ptr<zmqpp::socket>> receivers; //index receivers

// ******************* ROS subscribe and publish *************************
// 存放订阅和发布的主题
std::vector<ros::Subscriber> topic_subs;
std::vector<ros::Publisher> topic_pubs;

// ******************* send frequency control ***************************
// 存储每个发送操作的上一次发送时间。
std::vector<ros::Time> sub_t_last;
// 存储每个发送操作在一个周期内的发送消息数量。
std::vector<int> send_num;
// 控制发布频率
bool send_freq_control(int i);

// ****************** launch receive threads *****************************
// 用于存储每个接收线程的标志和上一次的标志。
std::vector<bool> recv_thread_flags;
std::vector<bool> recv_flags_last;
// 存储接收线程对象。
std::vector<std::thread> recv_threads;
void recv_func(int i);

// ***************** stop send/receive ******************************
// 停止特定索引的发送或接收操作。
void stop_send(int i);
void stop_recv(int i);

#endif