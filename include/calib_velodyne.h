#ifndef CALIB_VELO_H
#define CALIB_VELO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_types.h>
#include <actionlib/client/simple_action_client.h>
#include <collect_calib_data/calib_veloAction.h>

namespace CalibraionVelodyne
{
    class CalibVelo {

    private:

        std::mutex mtx_;

        actionlib::SimpleActionClient<collect_calib_data::calib_veloAction> client;
        collect_calib_data::calib_veloResultConstPtr result;
        collect_calib_data::calib_veloGoal goal;

        float x_initial_;
        float y_initial_;
        float z_initial_;

        unsigned int count_;
        float pitch_;
        bool send_deg_;

        std::string dir_path_;
        std::string file_name_;

        ros::Subscriber cloud_sub_;
        ros::Publisher pub_;
        
        tf2_ros::Buffer tfBuffer_;
        geometry_msgs::TransformStamped ts_;

        void get_params();
        void get_ts(
            geometry_msgs::TransformStamped &ts);
        void getpc2_cb(
            const sensor_msgs::PointCloud2 &cloud_msgs);

    public:

        CalibVelo(ros::NodeHandle &nh);
        ~CalibVelo();
        void run();

    };
}

#endif
