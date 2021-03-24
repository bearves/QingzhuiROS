#ifndef QZ_ROBOT_STATE_PUBLISHER_H
#define QZ_ROBOT_STATE_PUBLISHER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "qz_bridge/GaitPhase.h"
#include "qz_bridge/RobotTipState.h"

#include <unistd.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>

#include "DataClient.h"
#include "DataHost.h"

#define NON_RT
#include <aris.h>

#define MULTI_IMU
#define IMU_INDEX 3

namespace qz_bridge
{
    class QzRobotStatePublisher
    {
    public:
        QzRobotStatePublisher();
        ~QzRobotStatePublisher();
        void update();

    private:
        robot_libs::networking::DataClient client;
        robot_libs::networking::DataHost data_host;
        ros::Publisher odom_pub;
        ros::Publisher imu_pub;
        ros::Publisher gaitphase_pub;
        ros::Publisher tipstate_pub;

        void publishOdomMsg();
        void publishImuMsg();
        void publishTipStateMsg();
        void publishGaitPhaseMsg();

        void getPhaseData(
            boost::array<float, 6> &phase_data_entry,
            const rapidjson::Document *p_custom_data,
            const char *data_key_name);
        void getTipStateData(
            boost::array<double, 18> &tip_state_data_entry,
            const rapidjson::Document *p_custom_data,
            const char *data_key_name);
    };

}

#endif