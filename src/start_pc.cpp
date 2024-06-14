#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sys/utsname.h>

using namespace std;
// 0待机，1启动，2停止
int order_list[3] = {0, 1, 2};
int order_a = 0, order_b = 0, order_c = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "start_pc");
    ros::NodeHandle nh;
    //   识别中文
    setlocale(LC_ALL, "");
    // 接收特定启动指令，并运行制定位置的命令shell文件，或者启动指定的指令
    ROS_INFO("车辆处于待机状态，等待启动。0待机，1启动，2停止。");
    ros::Publisher start_order_pc = nh.advertise<swarm_ros_bridge::start>("start_order", 1);

    while (ros::ok())
    {
        swarm_ros_bridge::start msg;
        // 读取键盘输入
        char car;
        cout << "请输入接收指令的车辆（a、b、c）：";
        cin >> car;
        // 根据不同车辆执行不同操作
        if (car == 'a')
        {
            cout << "请输入" << car << "车的指令：";
            cin >> order_a;
            // 检查输入是否在指定的指令列表中
            if (std::find(std::begin(order_list), std::end(order_list), order_a) == std::end(order_list))
            {
                ROS_ERROR("输入指令不合法！");
                break;
            }
            msg.uav_name = "a";
            msg.index_a = order_a;
            msg.index_b = order_b;
            msg.index_c = order_c;
        }
        else if (car == 'b')
        {
            cout << "请输入" << car << "车的指令：";
            cin >> order_b;
            // 检查输入是否在指定的指令列表中
            if (std::find(std::begin(order_list), std::end(order_list), order_b) == std::end(order_list))
            {
                ROS_ERROR("输入指令不合法！");
                break;
            }
            msg.uav_name = "b";
            msg.index_a = order_a;
            msg.index_b = order_b;
            msg.index_c = order_c;
        }
        else if (car == 'c')
        {
            cout << "请输入" << car << "车的指令：";
            cin >> order_c;
            // 检查输入是否在指定的指令列表中
            if (std::find(std::begin(order_list), std::end(order_list), order_c) == std::end(order_list))
            {
                ROS_ERROR("输入指令不合法！");
                break;
            }
            msg.uav_name = "c";
            msg.index_a = order_a;
            msg.index_b = order_b;
            msg.index_c = order_c;
        }
        else
        {
            ROS_ERROR("无效的车辆指令！");
            break;
        }

        start_order_pc.publish(msg);
        ros::spinOnce();
    }
}