#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include "./../include/calib_velodyne.hpp"

namespace CalibraionVelodyne
{
    CalibVelo::CalibVelo(ros::NodeHandle &nh) : 
        count_ (0),
        pub_ (nh.advertise<sensor_msgs::JointState>("ptu/cmd", 1)),
        cloud_sub_ (nh.subscribe("/velodyne_points", 1, &CalibVelo::getpc2_cb, this))
    {
        get_params();
    }

    CalibVelo::~CalibVelo()
    {
    }

    void CalibVelo::get_params()
    {
        ros::param::get("/calib_velodyne_ptu/dir_path", dir_path_);
        ros::param::get("/calib_velodyne_ptu/file_name", file_name_);
    }


    void CalibVelo::get_ts(
        geometry_msgs::TransformStamped &ts)
    {
    }

    void CalibVelo::getpc2_cb(
        const sensor_msgs::PointCloud2 &pc2)
    {
        sensor_msgs::PointCloud2 pc2_transformed;
        Eigen::Matrix4f R;
        //Eigen::Matrix4f R_inv;

        if(!send_deg_)
        {
            ROS_INFO("Now degree of mount : ");
        }
        else
        {
            R = tf2::transformToEigen(ts_.transform).matrix().cast<float>();
            //R_inv = R.transpose();

            pcl_ros::transformPointCloud(R, 
                                        pc2, 
                                        pc2_transformed);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>() );
            pcl::fromROSMsg(pc2_transformed, *cloud);

            std::string format = ".pcd";
            std::string savename = dir_path_ 
                                 + file_name_ 
                                 + std::to_string(count_) 
                                 + format; 
        
            pcl::io::savePCDFileASCII(savename, *cloud);
            ROS_INFO_STREAM("Save PCD file : " 
                            + file_name_ 
                            + std::to_string(count_) 
                            + format );
            count_++;
        }
    }

       
    void CalibVelo::run()
    {
        ros::Rate rate(1.0);
        float pitch;
        float tmp_deg;

        while(ros::ok())
        {
            ros::param::get("/calib_velodyne_ptu/pitch", pitch);
            if(pitch != tmp_deg)
            {
                tmp_deg = pitch;
                sensor_msgs::JointState js;
                js.header.stamp = ros::Time::now();
                js.name.resize(2);
                js.name[0] = "ptu_pan";
                js.name[1] = "ptu_tilt";
                js.position.resize(2);
                js.position[0] = 0.0;
                js.position[1] = pitch*(M_PI/180.0);
                pub_.publish(js);

                ROS_INFO("Wait for mount finished rotation ...");
                ros::Duration(2.0).sleep();
                get_ts(ts_);
                ros::spinOnce();
                send_deg_ = true;
            
                rate.sleep();
                ros::spinOnce();
            }
            else
            {
                //ROS_INFO("same degree");
                send_deg_ = false;
            
                rate.sleep();
                ros::spinOnce();
            }
        }
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calib_velodyne_ptu");
    ros::NodeHandle nh;

    CalibraionVelodyne::CalibVelo *get_pcl;
    get_pcl = new CalibraionVelodyne::CalibVelo(nh);
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
