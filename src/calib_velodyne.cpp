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
#include <pcl/common/transforms.h>

#include <actionlib/client/simple_action_client.h>
#include <collect_calib_data/calib_veloAction.h>

#include <mutex>

#include "./../include/calib_velodyne.hpp"

namespace CalibraionVelodyne
{
    CalibVelo::CalibVelo(ros::NodeHandle &nh) : 
        count_ (0),
        x_initial_ (0.0),
        y_initial_ (0.0),
        z_initial_ (0.0),
        pub_ (nh.advertise<sensor_msgs::JointState>("ptu/cmd", 1)),
        client ("/ptu_ts_server", true),
        cloud_sub_ (nh.subscribe("/velodyne_points", 1, &CalibVelo::getpc2_cb, this))
    {
        get_params();
        if(!client.waitForServer(ros::Duration(20.0)))
        {
            ROS_INFO("Could not find server : rotation_broadcaster");
            ros::shutdown();
        }
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
        goal.enable_work.data = true;
        client.sendGoal(goal);

        bool finished = client.waitForResult(ros::Duration(3.0));
        if(!finished)
        {

        }
        else
        {
            result = client.getResult();
            ts = result->result_rotation;
        }
    }

    void CalibVelo::getpc2_cb(
        const sensor_msgs::PointCloud2 &pc2)
    {
        sensor_msgs::PointCloud2 pc2_transformed;
        Eigen::Matrix4f R;

        if(!send_deg_)
        {
            ROS_INFO("Now degree of mount : ");
        }
        else
        {
            mtx_.lock();
            R = tf2::transformToEigen(ts_.transform).matrix().cast<float>();
            mtx_.unlock();

            ROS_INFO("X : %f", R(0,3));
            ROS_INFO("Y : %f", R(1,3));
            ROS_INFO("Z : %f", R(2,3));

            if(count_==0)
            {
                x_initial_ = R(0,3);
                y_initial_ = R(1,3);
                z_initial_ = R(2,3);
            }

            const float a = 0.8;
            const float b = 210.8;
            const float c = 2.5;
            const float d = 210.1;
            if(pitch_>0.0)
            {
                R(0,3) = R(0,3) - ((x_initial_-R(0,3))*a);
                R(2,3) = R(2,3) + ((z_initial_-R(2,3))*b);
            }
            else if(pitch_<0.0)
            {
                R(0,3) = R(0,3) + ((x_initial_-R(0,3))*c);
                R(2,3) = R(2,3) - ((z_initial_-R(2,3))*d);
            }

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
            send_deg_=false;
        }
    }

       
    void CalibVelo::run()
    {
        ros::Rate rate(1.0);
        float tmp_deg=12345.0;

        tf2_ros::TransformListener tfListener(tfBuffer_);

        while(ros::ok())
        {
            ros::param::get("/calib_velodyne_ptu/pitch", pitch_);
            if(pitch_ != tmp_deg)
            {
                tmp_deg = pitch_;
                sensor_msgs::JointState js;
                js.header.stamp = ros::Time::now();
                js.name.resize(2);
                js.name[0] = "ptu_pan";
                js.name[1] = "ptu_tilt";
                js.position.resize(2);
                js.position[0] = 0.0;
                js.position[1] = pitch_*(M_PI/180.0);
                js.velocity.resize(2);
                js.velocity[0] = 0.6;
                js.velocity[1] = 0.6;
                pub_.publish(js);

                ROS_INFO("Wait for mount finished rotation ...");
                ros::Duration(5.0).sleep();

                mtx_.lock();
                get_ts(ts_);
                send_deg_ = true;
                mtx_.unlock();
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
