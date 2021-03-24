#include "ros/ros.h"
#include "QzRobotStatePublisher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qz_state_publisher");
    qz_bridge::QzRobotStatePublisher publisher;
    ros::Rate loop_rate(100);
    int count = 0;

    // pull robot state data from dataAgent and publish them to related ros nodes
    while(ros::ok())
    {
        ros::spinOnce();
        publisher.update();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
