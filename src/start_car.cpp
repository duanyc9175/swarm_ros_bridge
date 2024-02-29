#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <swarm_ros_bridge/start.h>

using namespace std;
// 定义文件路径
string file_start = "/swarm_ros_bridge/src/swarm_ros_bridge/shell/rs2velodyne.sh";
string file = std::getenv("HOME") + file_start;
std::string command = "chmod 777 " + file;


void order_action(const int order , const string name)
{
    string robot_now = name;
    switch (order)
    {
    case 0:
        // robot_now车辆待机
        ROS_INFO("%s车辆待机", robot_now.c_str());
        break;
    case 1:
        ROS_INFO("%s车辆启动", robot_now.c_str());
        // 设置文件相对路径
        ROS_INFO("启动文件路径：%s", file.c_str());
        // 赋予file文件权限
        system(command.c_str());
        // 调用shell文件
        popen(file.c_str(), "r");
        ROS_INFO("启动文件执行完毕");
        break;
    case 2:
        ROS_INFO("车辆停止");
        // 关闭所有终端
        system("killall gnome-terminal-server");
        break;
    default:
        ROS_INFO("未知指令");
        break;
    }
}



void startCallback(const swarm_ros_bridge::start::ConstPtr &msg)
{
    string name_a = "a", name_b = "b", name_c = "c";
    // a车消息
    int order_a = msg->index_a;
    order_action(order_a, name_a);
    // b车消息
    int order_b = msg->index_b;
    order_action(order_b, name_b);
    // c车消息
    int order_c = msg->index_c;
    order_action(order_c, name_c);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_car");
    ros::NodeHandle nh;
    //   识别中文
    setlocale(LC_ALL, "");
    // 接收特定启动指令，并运行制定位置的命令shell文件，或者启动指定的指令
    ROS_INFO("终端指令等待");
    ros::Subscriber start_order = nh.subscribe<swarm_ros_bridge::start>("start_order", 1, startCallback);
    
    // 如果没有发布者，等待5s
    while (start_order.getNumPublishers() == 0)
    {
        ROS_INFO("等待发布者");
        ros::Duration(1.0).sleep();
    }

    ros::spin();

    return 0;
}