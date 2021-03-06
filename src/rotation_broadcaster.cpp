#include <Eigen/Core>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <collect_calib_data/rotation_broadcasterAction.h>

#include "./../include/rotation_broadcaster.hpp"

namespace CalibrationVelodyne
{

    RotationBroadcaster::RotationBroadcaster(ros::NodeHandle &nh) :
        success (false),
        server (nh, "rotation_broadcaster", false),
        degree_sub (nh.subscribe("degree", 1, &RotationBroadcaster::degree_cb, this))
    {
        ros::param::get("/rotation_broadcaster/x", x);
        ros::param::get("/rotation_broadcaster/y", y);
        ros::param::get("/rotation_broadcaster/z", z);

        server.registerGoalCallback(boost::bind(&RotationBroadcaster::goal_callback, this));
        server.registerPreemptCallback(boost::bind(&RotationBroadcaster::preempt_callback, this));
        server.start();
    } 


    RotationBroadcaster::~RotationBroadcaster()
    {
    }


    // 回転行列Rの初期位置を設定
    void RotationBroadcaster::set_R(
                Eigen::Matrix3f &R,
                const float roll,
                const float pitch,
                const float yaw)
    {
        Eigen::Matrix3f R_tmp;
        Eigen::Matrix3f Rpi;
        Eigen::Matrix3f Rya;
        Eigen::Matrix3f Rro;

        Rro << 1, 0, 0,  
               0, cos(roll), -sin(roll),  
               0, sin(roll), cos(roll); 

        Rya << cos(yaw), 0, sin(yaw), 
               0, 1, 0, 
               -sin(yaw), 0, cos(yaw);

        Rpi << cos(pitch), -sin(pitch), 0, 
               sin(pitch), cos(pitch), 0, 
               0, 0, 1;

        R = Rro * Rpi * Rya;
        //R = R_tmp.inverse();

        //std::cout << "R : \n" << R << std::endl; 
        feedback.work.data = true;
        feedback.work_method.data = "Done set rotation matrix";
        server.publishFeedback(feedback);
    }


    Eigen::Vector3f RotationBroadcaster::set_diff_xyz(
        const float roll,
        const float pitch,
        const float yaw,
        const float x,
        const float y,
        const float z)
    {
        Eigen::Matrix3f R_3x3;
        Eigen::Vector3f t;
        t << x, y, z;

        set_R(R_3x3, roll, pitch, yaw);
        t = R_3x3*t;

        return t;
    }


    void RotationBroadcaster::set_ts(
        const float roll,
        const float pitch,
        const float yaw,
        geometry_msgs::TransformStamped &ts)
    {
        Eigen::Vector3f t;
        t = set_diff_xyz(x, y, z, roll, pitch, yaw);

        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "mount";
        ts.child_frame_id = "velodyne";

        tf2::Vector3 translation;
	    translation.setValue(t(0), t(1), t(2));
	    ts.transform.translation.x = translation.x();
	    ts.transform.translation.y = translation.y();
	    ts.transform.translation.z = translation.z();

	    tf2::Quaternion rotation;
	    rotation.setRPY(roll, pitch, yaw);
	    ts.transform.rotation.x = rotation.x();
	    ts.transform.rotation.y = rotation.y();
	    ts.transform.rotation.z = rotation.z();
	    ts.transform.rotation.w = rotation.w();
        
        feedback.work.data = true;
        feedback.work_method.data = "Done set TransformStamp";
        server.publishFeedback(feedback);
    }

    void RotationBroadcaster::degree_cb(
        const std_msgs::Int16 &potentio_cb)
    {
        if(degree != potentio_cb.data)
        {
            degree = potentio_cb.data;
            cb_success = true;
        }
        else
        {
            cb_success = false;
        }
    }


    void RotationBroadcaster::goal_callback()
    {
        if(server.isNewGoalAvailable())
        {
          goal = server.acceptNewGoal();
        }

        ros::Rate rate(1.0);
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped ts;

        while(ros::ok())
        {
            try
            {
                ts = tfBuffer.lookupTransform("map", "velodyne", ros::Time());
                break;
            }
            catch(tf2::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                ros::spinOnce();
                continue;
            }

            /* 
            if(cb_success)
            {
                pitch = degree * (M_PI/180.0);
                set_ts(0.0, pitch, 0.0, ts);
                cb_success = false;
                break;
            }
            else
            {
                ROS_WARN("Do not movement the mount");
                //ROS_WARN("potentio degree : %d", degree);
                rate.sleep();
                ros::spinOnce();
            }
            */
        }
        
        result.success_work.data = true;
        result.result_deg.data = degree;
        result.result_rotation = ts;
        server.setSucceeded(result, "ts");
    }


    void RotationBroadcaster::preempt_callback()
    {
        result.success_work.data = false; 
        server.setPreempted(result, "preempted");
    }


    void RotationBroadcaster::run()
    {
        ros::spin();
    }

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rotation_broadcaster");
    ros::NodeHandle nh;

    CalibrationVelodyne::RotationBroadcaster *server;
    server = new CalibrationVelodyne::RotationBroadcaster(nh);
    server->run();
    delete server;

    return 0;
}
