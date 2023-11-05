#include <iostream> //used for testing
#include "ros/ros.h"
#include "../include/state_machine.hpp"
//#include "../include/locate_elg.hpp"
#include <yaml-cpp/yaml.h>
//#include "../include/PID.hpp"
#include "tf/transform_datatypes.h"
#include <math.h>
#include <tf/tf.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace sml = boost::sml;
using namespace sml;
using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control_test");
    ros::NodeHandle nh;
    drone::dependencies d;
    ros::Rate rate(10);
    d.n = nh;
    sml::sm<drone::icarus> sm{d, rate};
    // while(!d.boot_cmd)
    // {   
    //     ROS_INFO("Waiting for Boot");
    //     ros::spinOnce();
    //     sleep(1);
    // }
    
    sleep(5);
    ROS_INFO("5s left!!");
    sleep(5);
    ROS_INFO("Mission Start!");
    drone::TFtarget target;


    // geometry_msgs::Quaternion rotate;
    // tf::Quaternion tf_rotate; 

    //延时10秒起飞
    // sleep(10);

    sm.process_event(drone::release{});
    //起飞到巡航高度1m
    for( double i = 1 ; i <= 5*10 ; i++ )
    {
        target.SetTarget( 0 , 0 , 0 , i/50 * 1.6 );
        sm.process_event(drone::takeoff{target});
        rate.sleep();
        ros::spinOnce();
    }

    while (sm.is("takingoff"_s))
    {   
        sm.process_event(drone::takeoffover{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("taking_off");
    }
    cv::VideoCapture cap(0);
	cv::Mat image;  
	cap >> image;
    cv::imwrite("/home/drone/acfly_ws/src/linetracing/img/gao.jpg", image); 

    //测试用


    //开始进入巡航过程


    // for( double i = 1 ; i <= 10*5 ; i++ )
    // {
    //     target.SetTarget( 0 ,  i/25.0 , 0 , 1 );
    //     sm.process_event(drone::moveTo{target});
    //     rate.sleep();
    //     ros::spinOnce();
    // }



    //目标1
    // target.SetTarget(0, 0, 0, 1 , 0.8, 0 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_1");
    // }
    // //目标2
    // target.SetTarget(0, 0, 0, 1 , 1.6, 0 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_2");
    // }
    // //目标3
    // target.SetTarget(0, 0, 0, 1 , 2.4, 0 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_3");
    // }
    // //目标4
    // target.SetTarget(0, 0, 0, 1 , 3.2, 0 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_4");
    // }
    // //目标5
    // target.SetTarget(0, 0, 0, 1 , 3.2, -0.8 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_5");
    // }
    // //目标6
    // target.SetTarget(0, 0, 0, 1 , 3.2, -1.6 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_6");
    // }
    // //目标7
    // target.SetTarget(0, 0, 0, 1 , 3.2, -2.4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_7");
    // }
    // //目标8
    // target.SetTarget(0, 0, 0, 1 , 3.2, -3.2 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_8");
    // }
    // //目标9
    // target.SetTarget(0, 0, 0, 1 , 3.2, -4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_9");
    // }
    // //目标10
    // target.SetTarget(0, 0, 0, 1 , 2.4, -4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_10");
    // }
    // //目标11
    // target.SetTarget(0, 0, 0, 1 , 1.6, -4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_11");
    // }
    // //目标12
    // target.SetTarget(0, 0, 0, 1 , 0.8, -4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_12");
    // }
    // //目标13
    // target.SetTarget(0, 0, 0, 1 , 0, -4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_13");
    // }
    // //目标14
    // target.SetTarget(0, 0, 0, 1 , 0, -3.2 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_14");
    // }
    // //目标15
    // target.SetTarget(0, 0, 0, 1 , 0, -2.4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_15");
    // }
    // //目标16
    // target.SetTarget(0, 0, 0, 1 , 0, -1.6 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_16");
    // }
    // //目标17
    // target.SetTarget(0, 0, 0, 1 , 0, -0.8 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_17");
    // }
    // //目标18
    // target.SetTarget(0, 0, 0, 1 , 0.8, -0.8 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_18");
    // }
    // //目标19
    // target.SetTarget(0, 0, 0, 1 , 0.8, -1.6 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_19");
    // }
    // //目标20
    // target.SetTarget(0, 0, 0, 1 , 0.8, -2.4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_20");
    // }
    // //目标21
    // target.SetTarget(0, 0, 0, 1 , 0.8, -3.2 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_21");
    // }
    // //目标22
    // target.SetTarget(0, 0, 0, 1 , 1.6, -3.2 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_22");
    // }
    // //目标23
    // target.SetTarget(0, 0, 0, 1 , 2.4, -3.2 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_23");
    // }
    // //目标24
    // target.SetTarget(0, 0, 0, 1 , 2.4, -2.4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_24");
    // }
    // //目标25
    // target.SetTarget(0, 0, 0, 1 , 2.4, -1.6 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_25");
    // }
    // //目标26
    // target.SetTarget(0, 0, 0, 1 , 2.4, -0.8 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_26");
    // }
    // //目标27
    // target.SetTarget(0, 0, 0, 1 , 1.6, -0.8 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_27");
    // }
    // //目标28
    // target.SetTarget(0, 0, 0, 1 , 1.6, -1.6 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_28");
    // }
    // //目标29
    // target.SetTarget( 0, 0, 0, 1 , 1.6, -2.4 , 1.7 );
    // while (sm.is("moving"_s))
    // {
    //     sm.process_event(drone::tickOnce{});
    //     rate.sleep();
    //     ros::spinOnce();
    //     ROS_INFO("moving_to_target_29");
    // }
    target.SetTarget(0, 0, 0, 1 , 0 , 0 , -0.1 );
    sm.process_event(drone::land{target});
    while (sm.is("landing"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("Landing");
    }
    sm.process_event(drone::To_lock{});
    return 0;
}
