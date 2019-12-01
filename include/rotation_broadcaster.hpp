#ifndef _ROTATION_BROADCASTER_H_
#define _ROTATION_BROADCASTER_H_

#include <Eigen/Core>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <collect_calib_data/rotation_broadcasterAction.h>

namespace CalibrationVelodyne
{
    struct diff_xyz 
    {
        float diff_x;
        float diff_y;
        float diff_z;
    };


    class RotationBroadcaster
    {
    private:

        float pitch;
        float x;
        float y;
        float z;
        bool success;
        
        ros::Subscriber degree_sub;

        actionlib::SimpleActionServer<collect_calib_data::rotation_broadcasterAction> server;
        collect_calib_data::rotation_broadcasterFeedback feedback;
        collect_calib_data::rotation_broadcasterResult result;
        collect_calib_data::rotation_broadcasterGoalConstPtr goal;


        void degree_cb(const std_msgs::Float32 &pitch_cb);

        void set_R(
            Eigen::Matrix3f &R,
            const float roll,
            const float pitch,
            const float yaw);

        Eigen::Vector3f set_diff_xyz(
            const float roll,
            const float pitch,
            const float yaw,
            const float x,
            const float y,
            const float z);

        void set_ts(
            const float roll,
            const float pitch,
            const float yaw,
            geometry_msgs::TransformStamped &ts);

        void goal_callback();
        void preempt_callback();

    public:

        RotationBroadcaster(ros::NodeHandle &nh);
        ~RotationBroadcaster();
        void run();
    };
}

#endif
