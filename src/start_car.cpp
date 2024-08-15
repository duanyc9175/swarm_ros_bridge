#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <sys/utsname.h>

using namespace std;
std_msgs::Int64 is_connected_msg;
// 定义文件路径
// string file_start_a = "/swarm_ros_bridge/src/swarm_ros_bridge/shell/one-click_a.sh";
// string file_start_b = "/swarm_ros_bridge/src/swarm_ros_bridge/shell/one-click_b.sh";
// string file_start_c = "./one-click.sh";
// string file_a = std::getenv("HOME") + file_start_a;
// string file_b = std::getenv("HOME") + file_start_b;
// string file_c = std::getenv("HOME") + file_start_c;

// string file_auto_sh = "/swarm_ros_bridge/src/swarm_ros_bridge/shell/Order/auto_drive.sh";
// string file_path = "/swarm_ros_bridge/src/swarm_ros_bridge/shell/Order";
// string file_auto = std::getenv("HOME") + file_auto_sh;

// 定义赋予权限的命令
// std::string command = "chmod 777 " + file_a + " " + file_b + " " + file_c;
std::string command = "gnome-terminal -- bash -c 'cd  &&sh ./one-click.sh'";

void Callback_A(const std_msgs::Int8::ConstPtr &msg)
{
    int order = msg->data;
    switch (order)
    {
    case 0:
        ROS_INFO("A车辆待机");
        break;
    case 1:
        system(command.c_str()); // 执行shell文件
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

void Callback_B(const std_msgs::Int8::ConstPtr &msg)
{

    int order = msg->data;
    switch (order)
    {
    case 0:
        ROS_INFO("B车辆待机");
        break;
    case 1:
        system(command.c_str()); // 执行shell文件
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

void Callback_C(const std_msgs::Int8::ConstPtr &msg)
{

    int order = msg->data;
    switch (order)
    {
    case 0:
        ROS_INFO("C车辆待机");
        break;
    case 1:
        system(command.c_str()); // 执行shell文件
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_car");
    ros::NodeHandle nh;
    //   识别中文
    setlocale(LC_ALL, "");
    is_connected_msg.data = 0;

    // 接收特定启动指令，并运行制定位置的命令shell文件，或者启动指定的指令
    ROS_INFO("终端指令等待");
    ros::Subscriber CMD_A = nh.subscribe<std_msgs::Int8>("cmd_A", 1, Callback_A);
    ros::Subscriber CMD_B = nh.subscribe<std_msgs::Int8>("cmd_B", 1, Callback_B);
    ros::Subscriber CMD_C = nh.subscribe<std_msgs::Int8>("cmd_C", 1, Callback_C);

    // 持续发布是否连接信息
    ros::Publisher is_connected = nh.advertise<std_msgs::Int64>("is_connected", 1);

    // 如果没有发布者，等待5s
    if (!CMD_A.getNumPublishers() || !CMD_B.getNumPublishers() || !CMD_C.getNumPublishers())
    {
        ROS_INFO("等待发布者");
    }
    ros::Rate loop_rate(10); // 10hz
    while (ros::ok())
    {
        is_connected.publish(is_connected_msg);
        is_connected_msg.data++;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}