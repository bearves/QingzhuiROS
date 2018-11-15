#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "DataClient.h"
#include "DataHost.h"
#include <aris.h>
#include <unistd.h>
#include <iostream>
#include <cstdio>

static const char fmt[] = "tt --online --load=%d --start=%d --stop=%d --trot_vel=%.3f --turn_rate=%.3f";
static const int LOAD = 9;

robot_libs::networking::DataClient client;
robot_libs::networking::DataHost data_host;
char cmd_buffer[300];
bool move_robot = false;

void cmdVelCallback(const geometry_msgs::Twist &msg)
{
    ROS_INFO("Get cmd: linear:  [%.3f %.3f %.3f]", msg.linear.x, msg.linear.y, msg.linear.z);
    ROS_INFO("         angular: [%.3f %.3f %.3f]", msg.angular.x, msg.angular.y, msg.angular.z);

    if (move_robot && (fabs(msg.linear.x) >= 0.02 || fabs(msg.angular.z) >= 0.02))
    {
        sprintf(cmd_buffer, fmt, LOAD, 1, 0, msg.linear.x, msg.angular.z);
    }
    else
    {
        sprintf(cmd_buffer, fmt, LOAD, 0, 1, 0.0, 0.0);
    }

    std::string cmd = cmd_buffer;

    int ret = client.SendCmd(cmd);
    if (ret)
        ROS_INFO("send command successful");
    else
        ROS_INFO("error happened when sending request");
}

bool cmdStartCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    move_robot = req.data;
    if (req.data)
    {
        res.message = "Start to move robot";
    }
    else
    {
        res.message = "Stop to move robot";
        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0;
        stop_msg.angular.z = 0;
        pub.publish(stop_msg);
        //ros::spinOnce();
        ROS_INFO("Send a stop msg immediately");
    }

    res.success = true;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_bridge");

    ros::NodeHandle n("~");
    std::string param;

    if (n.hasParam("robotconfig"))
    {
        n.getParam("robotconfig", param);
        ROS_INFO("Use specified config file path: %s", param.c_str());
    }
    else
    {
        param = "./RobotY13.xml";
        ROS_INFO("Use default config file path: %s", param.c_str());
    }

    ros::Subscriber sub = n.subscribe("cmd_vel", 10, cmdVelCallback);
    ros::ServiceServer srv = n.advertiseService("cmd_start", cmdStartCallback);

    client.Initialize(param, "RosCmdVelBridge");

    ros::spin();

    return 0;
}
