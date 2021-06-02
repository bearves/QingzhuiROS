#include "QzRobotStatePublisher.h"

namespace qz_bridge
{
    QzRobotStatePublisher::QzRobotStatePublisher()
    {
        ros::NodeHandle n("~");

        if (n.hasParam("robotconfig"))
        {
            n.getParam("robotconfig", config_file_path);
            ROS_INFO("Use specified config file path: %s", config_file_path.c_str());
        }
        else
        {
            config_file_path = "./RobotY13.xml";
            ROS_INFO("Use default config file path: %s", config_file_path.c_str());
        }

        odom_pub = n.advertise<nav_msgs::Odometry>("robot_odom", 10);
        imu_pub = n.advertise<sensor_msgs::Imu>("robot_imu", 10);
        gaitphase_pub = n.advertise<GaitPhaseMsg>("robot_gait_phase", 10);
        tipstate_pub = n.advertise<RobotTipStateMsg>("robot_tip_state", 10);

        client.Initialize(config_file_path, "RosQzStatePublisher");
    }

    QzRobotStatePublisher::~QzRobotStatePublisher()
    {
    }

    void QzRobotStatePublisher::update()
    {
        client.GetData(robot_libs::networking::RobotDataType::ROBOT_STATE, data_host);
        client.GetData(robot_libs::networking::RobotDataType::CUSTOM_GAIT_DATA, data_host);

        publishOdomMsg();
        publishImuMsg();
        publishGaitPhaseMsg();
        publishTipStateMsg();
    }

    void QzRobotStatePublisher::publishOdomMsg()
    {
        double body_est_vel[3]{0, 0, 0};
        double body_est_pos[3]{0, 0, 0};
        nav_msgs::Odometry msg;

        auto *p_custom_data = data_host.GetCustomGaitData()->data();

        if (p_custom_data && p_custom_data->IsObject())
        {
            if (p_custom_data->HasMember("bodyEst"))
            {
                rapidjson::Value &body_est_data = (*p_custom_data)["bodyEst"];
                if (body_est_data.IsArray())
                {
                    for (int i = 0; i < 3; i++)
                    {
                        body_est_vel[i] = body_est_data[i].GetDouble();
                        body_est_pos[i] = body_est_data[i + 3].GetDouble();
                    }
                }
            }
        }

        double body_est_angvel[3]{0, 0, 0};
        double body_est_ang[3]{0, 0, 0};
        auto imu_data = data_host.GetRobotStateData()->imu_data;
#ifdef MULTI_IMU
        auto imu_data_for_use = imu_data[IMU_INDEX];
#else
        auto imu_data_for_use = imu_data;
#endif
        for (int i = 0; i < 3; i++)
        {
            body_est_ang[i] = imu_data_for_use.euler[i];
            body_est_angvel[i] = imu_data_for_use.gyro[i];
        }

        //ROS_INFO("Vel: %.4f, %.4f Rate: %.4f", trot_vel, side_vel, turn_rate);

        msg.child_frame_id = "base_link";
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        msg.twist.twist.linear.x = body_est_vel[0];
        msg.twist.twist.linear.y = body_est_vel[1];
        msg.twist.twist.linear.z = body_est_vel[2];
        msg.twist.twist.angular.x = body_est_angvel[0];
        msg.twist.twist.angular.y = body_est_angvel[1];
        msg.twist.twist.angular.z = body_est_angvel[2];

        msg.pose.pose.position.x = body_est_pos[0];
        msg.pose.pose.position.y = body_est_pos[1];
        msg.pose.pose.position.z = body_est_pos[2];

        //convert to quaternion
        tf2::Quaternion q;
        q.setRPY(body_est_ang[0], body_est_ang[1], body_est_ang[2]);

        msg.pose.pose.orientation.x = q[0];
        msg.pose.pose.orientation.y = q[1];
        msg.pose.pose.orientation.z = q[2];
        msg.pose.pose.orientation.w = q[3];

        odom_pub.publish(msg);
    }

    void QzRobotStatePublisher::publishImuMsg()
    {
        double body_angvel[3]{0, 0, 0};
        double body_ang[3]{0, 0, 0};
        double body_acc[3]{0, 0, 0};
        auto imu_data = data_host.GetRobotStateData()->imu_data;
#ifdef MULTI_IMU
        auto imu_data_for_use = imu_data[IMU_INDEX];
#else
        auto imu_data_for_use = imu_data;
#endif
        for (int i = 0; i < 3; i++)
        {
            body_ang[i] = imu_data_for_use.euler[i];
            body_angvel[i] = imu_data_for_use.gyro[i];
            body_acc[i] = imu_data_for_use.accel[i];
        }

        sensor_msgs::Imu msg;
        msg.header.frame_id = "imu";
        msg.header.stamp = ros::Time::now();

        msg.linear_acceleration.x = body_acc[0];
        msg.linear_acceleration.y = body_acc[1];
        msg.linear_acceleration.z = body_acc[2];
        msg.angular_velocity.x = body_angvel[0];
        msg.angular_velocity.y = body_angvel[1];
        msg.angular_velocity.z = body_angvel[2];

        //convert to quaternion
        tf2::Quaternion q;
        q.setRPY(body_ang[0], body_ang[1], body_ang[2]);

        msg.orientation.x = q[0];
        msg.orientation.y = q[1];
        msg.orientation.z = q[2];
        msg.orientation.w = q[3];

        imu_pub.publish(msg);
    }

#ifdef BRIDGE_PROTOCOL_VERSION_1
    void QzRobotStatePublisher::publishGaitPhaseMsg()
    {
        GaitPhaseMsg msg;
        auto *p_custom_data = data_host.GetCustomGaitData()->data();

        getPhaseData(msg.stance_phase, p_custom_data, "stPgs");
        getPhaseData(msg.swing_phase, p_custom_data, "swPgs");
        getPhaseData(msg.touch_possibility, p_custom_data, "tchPgs");

        msg.header.frame_id = "base_link";
        msg.header.stamp = ros::Time::now();

        gaitphase_pub.publish(msg);
    }
#endif

#ifdef BRIDGE_PROTOCOL_VERSION_2
    void QzRobotStatePublisher::publishGaitPhaseMsg()
    {
        GaitPhaseMsg msg;
        auto *p_custom_data = data_host.GetCustomGaitData()->data();

        getPhaseData(msg.gait_count, p_custom_data, "tag");
        getPhaseData(msg.gait_phase, p_custom_data, "pgs");
        getPhaseData(msg.touch_possibility, p_custom_data, "tchPgs");

        msg.header.frame_id = "base_link";
        msg.header.stamp = ros::Time::now();

        gaitphase_pub.publish(msg);
    }
#endif

#ifdef BRIDGE_PROTOCOL_VERSION_1
    void QzRobotStatePublisher::publishTipStateMsg()
    {
        RobotTipStateMsg msg;
        auto *p_custom_data = data_host.GetCustomGaitData()->data();

        getTipStateData(msg.tip_pos, p_custom_data, "actTip");
        getTipStateData(msg.tip_vel, p_custom_data, "actTipV");
        getTipStateData(msg.tip_fce, p_custom_data, "actTipF");

        msg.header.frame_id = "base_link";
        msg.header.stamp = ros::Time::now();

        tipstate_pub.publish(msg);
    }
#endif

#ifdef BRIDGE_PROTOCOL_VERSION_2
    void QzRobotStatePublisher::publishTipStateMsg()
    {
        RobotTipStateMsg msg;
        auto *p_custom_data = data_host.GetCustomGaitData()->data();

        getTipStateData(msg.tip_pos, p_custom_data, "actTip");
        getTipStateData(msg.cmd_pos, p_custom_data, "cmdTip");
        getTipStateData(msg.tip_fce, p_custom_data, "actFoc");
        getTipStateData(msg.cmd_fce, p_custom_data, "cmdFoc");

        msg.header.frame_id = "base_link";
        msg.header.stamp = ros::Time::now();

        tipstate_pub.publish(msg);
    }
#endif

    void QzRobotStatePublisher::getTipStateData(
        boost::array<double, 18> &tip_state_data_entry,
        const rapidjson::Document *p_custom_data,
        const char *data_key_name)
    {
        if (p_custom_data->HasMember(data_key_name))
        {
            const rapidjson::Value &value = (*p_custom_data)[data_key_name];
            if (value.IsArray())
            {
                for (int i = 0; i < 18; i++)
                {
                    tip_state_data_entry[i] = value[i].GetDouble();
                }
            }
        }
    }

    void QzRobotStatePublisher::getPhaseData(
        boost::array<float, 6> &phase_data_entry,
        const rapidjson::Document *p_custom_data,
        const char *data_key_name)
    {
        if (p_custom_data->HasMember(data_key_name))
        {
            const rapidjson::Value &value = (*p_custom_data)[data_key_name];
            if (value.IsArray())
            {
                for (int i = 0; i < 6; i++)
                {
                    phase_data_entry[i] = value[i].GetDouble();
                }
            }
        }
    }

    void QzRobotStatePublisher::getPhaseData(
        boost::array<float, 1> &phase_data_entry,
        const rapidjson::Document *p_custom_data,
        const char *data_key_name)
    {
        if (p_custom_data->HasMember(data_key_name))
        {
            const rapidjson::Value &value = (*p_custom_data)[data_key_name];
            if (value.IsArray())
            {
                for (int i = 0; i < 1; i++)
                {
                    phase_data_entry[i] = value[i].GetDouble();
                }
            }
        }
    }

}