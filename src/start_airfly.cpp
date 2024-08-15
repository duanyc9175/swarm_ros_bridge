#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <sys/utsname.h>

using namespace std;
std_msgs::Int64 is_connected_msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_airfly");
    ros::NodeHandle nh;
    //识别中文
    setlocale(LC_ALL, "");
    ROS_INFO("启动airfly节点");
    is_connected_msg.data = 0;
    // 持续发布是否连接信息
    ros::Publisher is_connected = nh.advertise<std_msgs::Int64>("is_connected", 1);
    while (ros::ok())
    {
        is_connected.publish(is_connected_msg);
        ros::spinOnce();
    }
    return 0;
}
