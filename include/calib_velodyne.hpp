#ifndef CALIB_VELO_H
#define CALIB_VELO_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_types.h>

#include <actionlib/client/simple_action_client.h>
#include <collect_calib_data/rotation_broadcasterAction.h>

namespace CalibraionVelodyne
{
    class CalibVelo {

    private:

        actionlib::SimpleActionClient<collect_calib_data::rotation_broadcasterAction> client;
        collect_calib_data::rotation_broadcasterResultConstPtr result;
        collect_calib_data::rotation_broadcasterGoal goal;

        unsigned int count;
        bool send_degree;
        bool success_set_ts;
        int degree;
        int result_degree;

        std::string dir_path;
        std::string file_name;

        std::string cloud_frame;

        ros::Subscriber cloud_sub;
        ros::Publisher degree_pub;
        std_msgs::Int16 pub_degree;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

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
