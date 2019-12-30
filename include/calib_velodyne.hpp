#ifndef CALIB_VELO_H
#define CALIB_VELO_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/point_types.h>

namespace CalibraionVelodyne
{
    class CalibVelo {

    private:

        unsigned int count_;
        bool send_deg_;
        //int result_degree;

        std::string dir_path_;
        std::string file_name_;

        ros::Subscriber cloud_sub_;
        ros::Publisher pub_;
        
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
