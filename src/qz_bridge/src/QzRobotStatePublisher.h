#ifndef QZ_ROBOT_STATE_PUBLISHER_H
#define QZ_ROBOT_STATE_PUBLISHER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#ifdef BRIDGE_PROTOCOL_VERSION_1
#include "qz_bridge/GaitPhase.h"
#include "qz_bridge/RobotTipState.h"
#endif
#ifdef BRIDGE_PROTOCOL_VERSION_2
#include "qz_bridge/GaitPhaseV2.h"
#include "qz_bridge/RobotTipStateV2.h"
#endif

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
#ifdef BRIDGE_PROTOCOL_VERSION_1
    typedef qz_bridge::GaitPhase GaitPhaseMsg;
    typedef qz_bridge::RobotTipState RobotTipStateMsg;
#endif
#ifdef BRIDGE_PROTOCOL_VERSION_2
    typedef qz_bridge::GaitPhaseV2 GaitPhaseMsg;
    typedef qz_bridge::RobotTipStateV2 RobotTipStateMsg;
#endif

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
        std::string config_file_path;
        std::string version;

        void publishOdomMsg();
        void publishImuMsg();
        void publishTipStateMsg();
        void publishGaitPhaseMsg();

        void getPhaseData(
            boost::array<float, 6> &phase_data_entry,
            const rapidjson::Document *p_custom_data,
            const char *data_key_name);

        void getPhaseData(
            boost::array<float, 1> &phase_data_entry,
            const rapidjson::Document *p_custom_data,
            const char *data_key_name);

        void getTipStateData(
            boost::array<double, 18> &tip_state_data_entry,
            const rapidjson::Document *p_custom_data,
            const char *data_key_name);
    };

}

#endif