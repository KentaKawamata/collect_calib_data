#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <actionlib/client/simple_action_client.h>
#include <collect_calib_data/rotation_broadcasterAction.h>

#include "./../include/calib_velodyne.hpp"

namespace CalibraionVelodyne
{
    CalibVelo::CalibVelo(ros::NodeHandle &nh) : 
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        cloud_frame ("/velodyne_points"),
        count (0),
        client ("/rotation_broadcaster", true),
        degree_pub (nh.advertise<std_msgs::Int16>("servo", 10)),
        cloud_sub(nh.subscribe(cloud_frame, 1, &CalibVelo::getpc2_cb, this))
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
        ros::param::get("/calib_velodyne/dir_path", dir_path);
        ros::param::get("/calib_velodyne/file_name", file_name);
        ros::param::get("/calib_velodyne/degree", degree);
    }


    void CalibVelo::get_ts(
        geometry_msgs::TransformStamped &ts)
    {
        goal.enable_work.data = true;
        client.sendGoal(goal);

        bool finished = client.waitForResult(ros::Duration(3.0));
        if(!finished)
        {
            success_set_ts = false;

            ts.transform.rotation.x = 0.0;
            ts.transform.rotation.y = 0.0;
            ts.transform.rotation.z = 0.0;
            ts.transform.rotation.w = 1.0;
            ts.transform.translation.x = 0.0;
            ts.transform.translation.y = 0.0;
            ts.transform.translation.z = 0.0;
        }
        else
        {
            success_set_ts = true;
            result = client.getResult();
            ts = result->result_rotation;
            result_degree = result->result_deg.data;
        }
    }

    void CalibVelo::getpc2_cb(
        const sensor_msgs::PointCloud2 &pc2)
    {
        sensor_msgs::PointCloud2 pc2_transformed;
        geometry_msgs::TransformStamped ts;
        Eigen::Matrix4f R;

        if(!send_degree)
        {
            ROS_INFO("Now degree of mount : %d", result_degree);
        }
        else
        {
            get_ts(ts);
            if(!success_set_ts)
            {
                //ROS_WARN("Could not set ts");
                ROS_WARN("rotation broadcaster TimeOut");
            }
            else
            {
                R = tf2::transformToEigen(ts.transform).matrix().cast<float>();

                pcl_ros::transformPointCloud(R, 
                                             pc2, 
                                             pc2_transformed);

                pcl::fromROSMsg(pc2_transformed, *cloud);

                std::string format = ".pcd";
                std::string savename = dir_path 
                                     + file_name 
                                     + std::to_string(count) 
                                     + format; 
        
                pcl::io::savePCDFileASCII(savename, *cloud);
                count++;
                ROS_INFO_STREAM("Save PCD file : " 
                                + file_name 
                                + std::to_string(count) 
                                + format );
            }
        }
    }

       
    void CalibVelo::run()
    {
        ros::Rate rate(1.0);
        pub_degree.data = 1000.0;

        while(ros::ok())
        {
            ros::param::get("/calib_velodyne/degree", degree);
            if(degree != pub_degree.data)
            {
                ROS_INFO("Published degree to mount : %d", degree);
                pub_degree.data = degree;
                degree_pub.publish(pub_degree);
                send_degree = true;
            
                rate.sleep();
                ros::spinOnce();
            }
            else
            {
                //ROS_INFO("same degree");
                send_degree = false;
            
                rate.sleep();
                ros::spinOnce();
            }
        }
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calib_velodyne");
    ros::NodeHandle nh;

    CalibraionVelodyne::CalibVelo *get_pcl;
    get_pcl = new CalibraionVelodyne::CalibVelo(nh);
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
