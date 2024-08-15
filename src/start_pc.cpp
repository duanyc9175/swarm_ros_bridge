#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <sys/utsname.h>

using namespace std;
// 时间
ros::Time A_last_received_time, B_last_received_time, C_last_received_time, AirFly_last_received_time;
bool A_connection_active = true, B_connection_active = true, C_connection_active = true, AirFly_connection_active = true;

// 定时器回调函数，用于检查连接状态
void A_timerCallback(const ros::TimerEvent &)
{
    ros::Duration duration_since_last_msg = ros::Time::now() - A_last_received_time;
    double duration_sec = duration_since_last_msg.toSec();
    if (duration_sec >= 3.0) // 检查是否超过3秒
    {
        if (A_connection_active)
        {
            ROS_WARN("A connection lost");
            A_connection_active = false;
        }
    }
    else
    {
        if (!A_connection_active) // 重新收到消息，恢复连接
        {
            ROS_INFO("A connection restored.");
            A_connection_active = true;
        }
    }
}

void B_timerCallback(const ros::TimerEvent &)
{
    ros::Duration duration_since_last_msg = ros::Time::now() - B_last_received_time;
    double duration_sec = duration_since_last_msg.toSec();
    if (duration_sec >= 3.0) // 检查是否超过3秒
    {
        if (B_connection_active)
        {
            ROS_WARN("B connection lost");

            B_connection_active = false;
        }
    }
    else
    {
        if (!B_connection_active) // 重新收到消息，恢复连接
        {
            ROS_INFO("B connection restored.");
            B_connection_active = true;
        }
    }
}

void C_timerCallback(const ros::TimerEvent &)
{
    ros::Duration duration_since_last_msg = ros::Time::now() - C_last_received_time;
    double duration_sec = duration_since_last_msg.toSec();
    if (duration_sec >= 3.0) // 检查是否超过3秒
    {
        if (C_connection_active)
        {
            ROS_WARN("C connection lost");
            C_connection_active = false;
        }
    }
    else
    {
        if (!C_connection_active) // 重新收到消息，恢复连接
        {
            ROS_INFO("C connection restored.");
            C_connection_active = true;
        }
    }
}

void AirFly_timerCallback(const ros::TimerEvent &)
{
    ros::Duration duration_since_last_msg = ros::Time::now() - AirFly_last_received_time;
    double duration_sec = duration_since_last_msg.toSec();
    if (duration_sec >= 3.0) // 检查是否超过3秒
    {
        if (AirFly_connection_active)
        {
            ROS_WARN("AirFly connection lost");
            AirFly_connection_active = false;
        }
    }
    else
    {
        if (!AirFly_connection_active) // 重新收到消息，恢复连接
        {
            ROS_INFO("AirFly connection restored.");
            AirFly_connection_active = true;
        }
    }
}

void connect_stu_A(const std_msgs::Int64::ConstPtr &msg)
{
    // ROS_INFO("Received message from A: %ld", msg->data);
    A_last_received_time = ros::Time::now();
    if (!A_connection_active) // 检查连接是否从断开到恢复
    {
        ROS_INFO("A connection restored.");
        A_connection_active = true;
    }
}

void connect_stu_B(const std_msgs::Int64::ConstPtr &msg)
{
    // ROS_INFO("Received message from B: %ld", msg->data);
    B_last_received_time = ros::Time::now();
    if (!B_connection_active) // 检查连接是否从断开到恢复
    {
        ROS_INFO("B connection restored.");
        B_connection_active = true;
    }
}

void connect_stu_C(const std_msgs::Int64::ConstPtr &msg)
{
    // ROS_INFO("Received message from C: %ld", msg->data);
    C_last_received_time = ros::Time::now();
    if (!C_connection_active) // 检查连接是否从断开到恢复
    {
        ROS_INFO("C connection restored.");
        C_connection_active = true;
    }
}

void connect_stu_AirFly(const std_msgs::Int64::ConstPtr &msg)
{
    // ROS_INFO("Received message from AirFly: %ld", msg->data);
    AirFly_last_received_time = ros::Time::now();
    if (!AirFly_connection_active) // 检查连接是否从断开到恢复
    {
        ROS_INFO("AirFly connection restored.");
        AirFly_connection_active = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_pc");
    ros::NodeHandle nh;
    //   识别中文
    setlocale(LC_ALL, "");
    // 分别接收三个车发来的int64类型的消息
    ros::Subscriber sub_A = nh.subscribe("/A_is_connected", 1000, connect_stu_A);
    ros::Subscriber sub_B = nh.subscribe("/B_is_connected", 1000, connect_stu_B);
    ros::Subscriber sub_C = nh.subscribe("/C_is_connected", 1000, connect_stu_C);
    ros::Subscriber sub_airfly = nh.subscribe("/airfly_is_connected", 1000, connect_stu_AirFly);
    // 初始化最后接收消息的时间
    A_last_received_time = ros::Time::now();
    B_last_received_time = ros::Time::now();
    C_last_received_time = ros::Time::now();
    AirFly_last_received_time = ros::Time::now();

    // 创建定时器，每秒检查一次连接状态
    ros::Timer A_timer = nh.createTimer(ros::Duration(1.0), A_timerCallback);
    ros::Timer B_timer = nh.createTimer(ros::Duration(1.0), B_timerCallback);
    ros::Timer C_timer = nh.createTimer(ros::Duration(1.0), C_timerCallback);
    ros::Timer AirFly_timer = nh.createTimer(ros::Duration(1.0), AirFly_timerCallback);

    ros::spin();
}