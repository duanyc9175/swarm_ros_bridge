/**
 * @file bridge_node.cpp
 * @author Peixuan Shu (shupeixuan@qq.com)
 * @brief Reliable TCP bridge for ros data transfer in unstable network.
 * It will send/receive the specified ROS topics in ../config/ros_topics.yaml
 * It uses zmq socket(PUB/SUB mode), which reconnects others autonomously and
 * supports 1-N pub-sub connection even with TCP protocol.
 * 
 * Note: This program relies on ZMQPP (c++ wrapper around ZeroMQ).
 *  sudo apt install libzmqpp-dev
 * 
 * Core Idea: It would create the receving thread for each receiving ROS topic
 *  and send ROS messages in each sub_cb() callback.
 * 
 * @version 1.0
 * @date 2023-01-01
 * 
 * @license BSD 3-Clause License
 * @copyright (c) 2023, Peixuan Shu
 * All rights reserved.
 * 
 */

#include "bridge_node.hpp"

#include <swarm_ros_bridge/global_descriptor.h>
#include <swarm_ros_bridge/loop_info.h>
#include <swarm_ros_bridge/neighbor_estimate.h>
#include <swarm_ros_bridge/cloud_info.h>


// /* send messages frequency control */
// // this is the original freq control func that has been deprecated
// bool send_freq_control(int i)
// {
//   ros::Time t_now = ros::Time::now(); 
//   bool discard_flag;
//   if ((t_now - sub_t_last[i]).toSec() * sendTopics[i].max_freq < 1.0) {
//     discard_flag = true;
//   }
//   else {
//     discard_flag = false;
//     sub_t_last[i] = t_now; 
//   }
//   return discard_flag; // flag of discarding this message
// }

/* send messages frequency control */ //控制发送消息的频率
bool send_freq_control(int i)
{
  bool discard_flag;
  ros::Time t_now = ros::Time::now();      
  // check whether the send of this message will exceed the freq limit in the last period
  if ((send_num[i] + 1) / (t_now - sub_t_last[i]).toSec() > sendTopics[i].max_freq) {
    discard_flag = true;
  }
  else {
    discard_flag = false;
    send_num[i] ++;
  }
  // freq control period (1s)
  if ((t_now - sub_t_last[i]).toSec() > 1.0){
    sub_t_last[i] = t_now;
    send_num[i] = 0;
  }
  return discard_flag; // flag of discarding this message
}

/* uniform callback functions for ROS subscribers */
template <typename T, int i>
void sub_cb(const T &msg)
{
  /* frequency control */
  auto ignore_flag = send_freq_control(i);    //调用send_freq_control并将int i传入，传回布尔值，若为真，则函数直接返回，表示忽略这条消息
  if (ignore_flag){
    return; // discard this message sending, abort
  }

  /* serialize the sending messages into send_buffer */
  namespace ser = ros::serialization;              
  size_t data_len = ser::serializationLength(msg); // bytes length of msg
  std::unique_ptr<uint8_t> send_buffer(new uint8_t[data_len]);  // create a dynamic length array
  ser::OStream stream(send_buffer.get(), data_len);
  ser::serialize(stream, msg);              //将消息序列化，写入stream对象中    

  /* zmq send message */
  zmqpp::message send_array;                 //zmqpp::message可以包含多个数据帧，每个数据帧任意大小
  send_array << data_len;                    //将序列化后的数据长度加入该消息对象
  /* equal to:
    send_array.add_raw(reinterpret_cast<void const*>(&data_len), sizeof(size_t));
  */
  send_array.add_raw(reinterpret_cast<void const *>(send_buffer.get()), data_len);    //将数据本身加入该对象
  // std::cout << "ready send!" << std::endl;
  // send(&, true) for non-blocking, send(&, false) for blocking
  bool dont_block = false; // Actually for PUB mode zmq socket, send() will never block
  senders[i]->send(send_array, dont_block);
  // std::cout << "send!" << std::endl;



  // std::cout << msg << std::endl;
  // std::cout << i << std::endl;
}


/* uniform deserialize and publish the receiving messages */
template<typename T>                                                       
void deserialize_pub(uint8_t* buffer_ptr, size_t msg_size, int i)
{
  T msg;                                                     //创建消息对象，存储反序列化后的消息
  // deserialize the receiving messages into ROS msg
  namespace ser = ros::serialization;
  ser::IStream stream(buffer_ptr, msg_size);
  ser::deserialize(stream, msg);             //从stream读取消息到msg
  // publish ROS msg
  topic_pubs[i].publish(msg);          //发布msg到目标话题上
}


/* receive thread function to receive messages and publish them */
void recv_func(int i)
{
  while(recv_thread_flags[i])             //该标记控制线程的运行和停止
  {
    /* receive and process message */
    zmqpp::message recv_array;           //recv_array用于存储消息
    bool recv_flag; // receive success flag
    // std::cout << "ready receive!" << std::endl;
    // receive(&,true) for non-blocking, receive(&,false) for blocking
    bool dont_block = false; // 'true' leads to high cpu load       //表示是否使用非阻塞模式接收消息。如果为true，则不等待消息到达，如果为false，则等待消息到达。
    if (recv_flag = receivers[i]->receive(recv_array, dont_block))  //receivers[i]是套接字，从其接收消息存储到 recv_array中，dont_block为是否使用非阻塞模式接收消息，flag如果为真则说明
    {                                                               //接收到了消息，继续对接收数组进行解包和处理
      // std::cout << "receive!" << std::endl;
      size_t data_len;                                              
      recv_array >> data_len; // unpack meta data
      /*  equal to:
        recv_array.get(&data_len, recv_array.read_cursor++); 
        void get(T &value, size_t const cursor){
          uint8_t const* byte = static_cast<uint8_t const*>(raw_data(cursor)); 
          b = *byte;} 
      */
      // a dynamic length array by unique_ptr
      std::unique_ptr<uint8_t> recv_buffer(new uint8_t[data_len]);  
      // continue to copy the raw_data of recv_array into buffer
      memcpy(recv_buffer.get(), static_cast<const uint8_t *>(recv_array.raw_data(recv_array.read_cursor())), data_len);
      deserialize_publish(recv_buffer.get(), data_len, recvTopics[i].type, i);

      // std::cout << data_len << std::endl;
      // std::cout << recv_buffer.get() << std::endl;
    }

    /* if receive() does not block, sleep to decrease loop rate */
    if (dont_block)
      std::this_thread::sleep_for(std::chrono::microseconds(1000)); // sleep for us
    else
    {
      /* check and report receive state */
      if (recv_flag != recv_flags_last[i]){
        std::string topicName = recvTopics[i].name;
        if (topicName.at(0) != '/') {
          if (ns == "/") {topicName = "/" + topicName;}
          else {topicName = ns + "/" + topicName;}
        }  // print namespace prefix if topic name is not global
        ROS_INFO("[bridge node] \"%s\" received!", topicName.c_str());
      } // false -> true(first message in)        
      recv_flags_last[i] = recv_flag;
    }
  }
  return;
}

/* close recv socket, unsubscribe ROS topic */
void stop_send(int i)
{
  // senders[i]->unbind(std::string const &endpoint);
  senders[i]->close(); // close the send socket
  topic_subs[i].shutdown(); // unsubscribe
}

/* stop recv thread, close recv socket, unadvertise ROS topic */
void stop_recv(int i)
{
  recv_thread_flags[i] = false; // finish recv_func()
  // receivers[i]->disconnect(std::string &endpoint);
  receivers[i]->close(); // close the receive socket
  topic_pubs[i].shutdown(); // unadvertise
}

//TODO: generate or delete topic message transfers through a remote zmq service.

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;
  //设置命名空间
  ns = ros::this_node::getNamespace(); // namespace of this node

  std::cout << "--------[bridge_node]-------" << std::endl;
  std::cout << "namespaces=" << ns << std::endl;

  //************************ Parse configuration file **************************
  // get hostnames and IPs
  // 尝试从参数服务器中获取IP地址，保存在ip_xml中，如果获取失败则报错
  if (nh.getParam("IP", ip_xml) == false){
    ROS_ERROR("[bridge node] No IP found in the configuration!");
    return 1;
  }
  
  
  // get "send topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("send_topics", send_topics_xml)){
    // 检查参数类型是否为数组
    ROS_ASSERT(send_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    // 获取数组长度
    len_send = send_topics_xml.size();
  }
  else{
    // 如果没有获取到参数，则警告
    ROS_WARN("[bridge node] No send_topics found in the configuration!");
    // 数组长度置为0
    len_send = 0;
  }

  // 同理，获取接收主题的参数
  // get "receive topics" params (topic_name, topic_type, IP, port)
  if (nh.getParam("recv_topics", recv_topics_xml)){
    ROS_ASSERT(recv_topics_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
    len_recv = recv_topics_xml.size();
  }
  else{
    ROS_WARN("[bridge node] No recv_topics found in the configuration!");
    len_recv = 0;
  }

  // 如果数组长度超过了最大值，则报错
  if (len_send > SUB_MAX)
  {
    ROS_FATAL("[bridge_node] The number of send topics in configuration exceeds the limit %d!", SUB_MAX);
    return 2;
  }


  std::cout << "-------------IP------------" << std::endl;
  // 遍历一个名为ip_xml的映射（或字典），将其中的键值对（主机名和IP地址）打印出来
  for (auto iter = ip_xml.begin(); iter != ip_xml.end(); ++iter)
  {
    std::string host_name = iter->first;
    std::string host_ip = iter->second;
    std::cout << host_name << " : " << host_ip << std::endl;
    if (ip_map.find(host_name) != ip_map.end())
    { // ip_xml will never contain same names actually.
      ROS_WARN("[bridge node] IPs with the same name in configuration %s!", host_name.c_str());
    }
    // 将主机和IP地址的映射保存在ip_map中
    ip_map[host_name] = host_ip;
  }

  std::cout << "--------send topics--------" << std::endl;
  // std::set 为特殊容器，只存储唯一的元素，且按照升序排列。srcPorts存储源端口号
  std::set<int> srcPorts; // for duplicate check 
  // 遍历send_topics_xml数组，将其中的主题信息保存在sendTopics中
  for (int32_t i=0; i < len_send; ++i)
  {
    // 检查当前元素是否为结构体 
    ROS_ASSERT(send_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    // XmlRpc::XmlRpcValue是一个类，用于表示RPC方法的参数和结果。它是XML-RPC库的一部分，XML-RPC是一种使用HTTP作为传输协议，XML作为编码方式的远程过程调用协议
    XmlRpc::XmlRpcValue send_topic_xml = send_topics_xml[i];
    // 获取主题名、主题类型、最大频率、源IP地址、源端口号
    std::string topic_name = send_topic_xml["topic_name"];
    std::string msg_type = send_topic_xml["msg_type"];
    int max_freq = send_topic_xml["max_freq"];
    std::string srcIP = ip_map[send_topic_xml["srcIP"]];
    int srcPort = send_topic_xml["srcPort"];
    // 这行代码创建了一个名为TopicInfo的结构体实例，用于保存主题信息
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    sendTopics.emplace_back(topic);
    // check for duplicate ports:在检查是否有重复的端口。
    if (srcPorts.find(srcPort) != srcPorts.end()) {
      ROS_FATAL("[bridge_node] Send topics with the same srcPort %d in configuration!", srcPort);
      return 3;
    }
    srcPorts.insert(srcPort); // for duplicate check 
    // 确保话题的名称以斜杠开头，如果不是，则添加一个斜杠。
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  " << topic.max_freq << "Hz(max)" << std::endl;
  }

  std::cout << "-------receive topics------" << std::endl;
  for (int32_t i=0; i < len_recv; ++i)
  {
    ROS_ASSERT(recv_topics_xml[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue recv_topic_xml = recv_topics_xml[i];
    std::string topic_name = recv_topic_xml["topic_name"];
    std::string msg_type = recv_topic_xml["msg_type"];
    int max_freq = recv_topic_xml["max_freq"];
    std::string srcIP = ip_map[recv_topic_xml["srcIP"]];
    int srcPort = recv_topic_xml["srcPort"];
    TopicInfo topic = {.name=topic_name, .type=msg_type, .max_freq=max_freq, .ip=srcIP, .port=srcPort};
    recvTopics.emplace_back(topic);
    if (topic.name.at(0) != '/') {
      std::cout << ns;
      if (ns != "/") {std::cout << "/";}
    }  // print namespace prefix if topic.name is not global
    std::cout << topic.name << "  (from " << recv_topic_xml["srcIP"]  << ")" << std::endl;
  }

  // ********************* zmq socket initialize ***************************
  // send sockets (zmq socket PUB mode)
  for (int32_t i=0; i < len_send; ++i)
  {
    // 这行代码构造了一个URL，格式为"tcp://IP地址:端口号"。
    const std::string url = "tcp://" + sendTopics[i].ip + ":" + std::to_string(sendTopics[i].port);
    // 创建一个新的ZeroMQ PUB套接字。
    std::unique_ptr<zmqpp::socket> sender(new zmqpp::socket(context, zmqpp::socket_type::pub));
    sender->bind(url);
    // 使用std::move将套接字移动到senders向量中，以便稍后可以使用它来发送消息。
    senders.emplace_back(std::move(sender)); //sender is now released by std::move
  }

  // receive sockets (zmq socket SUB mode)
  for (int32_t i=0; i < len_recv; ++i)
  {
    const std::string url = "tcp://" + recvTopics[i].ip + ":" + std::to_string(recvTopics[i].port);
    std::string const zmq_topic = ""; // "" means all zmq topic
    std::unique_ptr<zmqpp::socket> receiver(new zmqpp::socket(context, zmqpp::socket_type::sub));
    // 使用 subscribe 函数订阅所有主题（在这种情况下，主题字符串为空，表示订阅所有主题）
    receiver->subscribe(zmq_topic);
    receiver->connect(url);
    receivers.emplace_back(std::move(receiver));
  }


  // ******************* ROS subscribe and publish *************************
  //ROS topic subsrcibe and send
  for (int32_t i=0; i < len_send; ++i)
  {
    // 对于每个主题，将当前时间添加到sub_t_last向量中，作为频率控制周期的开始时间。
    sub_t_last.emplace_back(ros::Time::now()); // freq control period start time
    send_num.emplace_back(0); // the send messages number in a period
    ros::Subscriber subscriber;
    // The uniform callback function is sub_cb()
    // 使用topic_subscriber函数订阅主题。这个函数需要主题名称、主题类型、节点句柄和索引作为参数。所有的回调函数都是sub_cb()。
    subscriber = topic_subscriber(sendTopics[i].name, sendTopics[i].type, nh_public, i);
    topic_subs.emplace_back(subscriber);
    // use topic_subs[i].shutdown() to unsubscribe
  }

  // ROS topic receive and publish
  for (int32_t i=0; i < len_recv; ++i) 
  {
    ros::Publisher publisher;
    publisher = topic_publisher(recvTopics[i].name, recvTopics[i].type, nh_public);
    topic_pubs.emplace_back(publisher);
  }

  // ****************** launch receive threads *****************************
  for (int32_t i=0; i < len_recv; ++i)
  {
    recv_thread_flags.emplace_back(true); // enable receive thread flags
    recv_flags_last.emplace_back(false); // receive success flag
    recv_threads.emplace_back(std::thread(&recv_func, i));
  }

  ros::spin();

  // ***************** stop send/receive ******************************
  for (int32_t i=0; i < len_send; ++i){
    stop_send(i);
  }

  for (int32_t i=0; i < len_recv; ++i){
    stop_recv(i);
  }
  
  return 0;
}
