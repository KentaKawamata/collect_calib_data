#ifndef _ROTATION_BROADCASTER_H_
#define _ROTATION_BROADCASTER_H_

#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <collect_calib_data/calib_veloAction.h>

namespace CalibrationVelodyne
{
    class ts_server
    {
    private:

        actionlib::SimpleActionServer<collect_calib_data::calib_veloAction> server_;
        collect_calib_data::calib_veloResult result_;
        collect_calib_data::calib_veloGoalConstPtr goal_;

        void goal_callback();
        void preempt_callback();

    public:

        ts_server(ros::NodeHandle &nh);
        ~ts_server();
        void run();
    };
}

#endif
