#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <collect_calib_data/calib_veloAction.h>

#include "./../include/calib_ts_server.h"

namespace CalibrationVelodyne
{

    ts_server::ts_server(ros::NodeHandle &nh) :
        server_ (nh, "calib_ptu_ts_server", false)
    {
        server_.registerGoalCallback(boost::bind(&ts_server::goal_callback, this));
        server_.start();
    } 


    ts_server::~ts_server()
    {
    }

    void ts_server::goal_callback()
    {
        if(server_.isNewGoalAvailable())
        {
          goal_ = server_.acceptNewGoal();
        }

        if(goal_->enable_work.data == false)
        {
            server_.shutdown();
            ros::shutdown();
        }

        ros::Rate rate(1.0);
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped ts;

        while(ros::ok())
        {
            try
            {
                ts = tfBuffer.lookupTransform("ptu_base_link", "velodyne", ros::Time(0), ros::Duration(2.0));
                break;
            }
            catch(tf2::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                ros::spinOnce();
                continue;
            }
        }
        
        result_.result_rotation = ts;
        server_.setSucceeded(result_, "ts");
    }


    void ts_server::preempt_callback()
    {
        server_.setPreempted(result_, "preempted");
    }


    void ts_server::run()
    {
        ros::spin();
    }

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calib_ts_server");
    ros::NodeHandle nh;

    CalibrationVelodyne::ts_server *server;
    server = new CalibrationVelodyne::ts_server(nh);
    server->run();
    delete server;

    return 0;
}
