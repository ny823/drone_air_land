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

#include <serial/serial.h>

#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include <../include/rec_red.hpp>


namespace sml = boost::sml;
using namespace sml;
using namespace std;
using namespace rec_red;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control_test");

    ros::NodeHandle nh;
    drone::dependencies d;
    bool is_catch_mission = false;
    int mission_num = -1;
    //is_catch_mission = ros::param::get("/mission_number",mission_num);


    ros::param::set("/LED",0);
    ros::param::set("/Throw",0);
    ros::param::set("/px",0);
    ros::param::set("/py",0);
    ros::Rate rate(10);
    d.n = nh;
    sml::sm<drone::icarus> sm{d, rate};

    while(d.boot_cmd!=true)
    {
        ros::spinOnce();
        sleep(1);
        cout<<"waiting boot..."<<endl;
    }
    //待修改，假设是任务1和任务2
    while(is_catch_mission == false || mission_num != 1 ||  mission_num != 2)
    {
        is_catch_mission = ros::param::get("/mission_number",mission_num);
    }
    if(mission_num == 1)
    {
    rate.sleep();
    drone::TFtarget target;
    .0
    ros::spinOnce();
    sm.process_event(drone::release{});
    //起飞到巡航高度1m
    for( double i = 1 ; i <= 5*10 ; i++ )
    {
        target.SetTarget( 0 , 0 , 0 , i/50 * 1.7 );
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
    // cv::VideoCapture cap(0);
	// cv::Mat image;  
	// cap >> image;
    // cv::imwrite("/home/drone/acfly_ws/src/linetracing/img/gao.jpg", image); 

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
    target.SetTarget(0, 0, 0, 1 , 0.8, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_1");
    }
    //目标2
    target.SetTarget(0, 0, 0, 1 , 1.6, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_2");
    }
    //目标3
    target.SetTarget(0, 0, 0, 1 , 2.4, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_3");
    }
    //目标4
    target.SetTarget(0, 0, 0, 1 , 3.2, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_4");
    }
    //目标5
    target.SetTarget(0, 0, 0, 1 , 3.2, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_5");
    }
    //目标6
    target.SetTarget(0, 0, 0, 1 , 3.2, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_6");
    }
    //目标7
    target.SetTarget(0, 0, 0, 1 , 3.2, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_7");
    }
    //目标8
    target.SetTarget(0, 0, 0, 1 , 3.2, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_8");
    }
    //目标9
    target.SetTarget(0, 0, 0, 1 , 3.2, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_9");
    }
    //目标10
    target.SetTarget(0, 0, 0, 1 , 2.4, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_10");
    }
    //目标11
    target.SetTarget(0, 0, 0, 1 , 1.6, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_11");
    }
    //目标12
    target.SetTarget(0, 0, 0, 1 , 0.8, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_12");
    }
    //目标13
    target.SetTarget(0, 0, 0, 1 , 0, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_13");
    }
    //目标14
    target.SetTarget(0, 0, 0, 1 , 0, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_14");
    }
    //目标15
    target.SetTarget(0, 0, 0, 1 , 0, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_15");
    }
    //目标16
    target.SetTarget(0, 0, 0, 1 , 0, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_16");
    }
    //目标17
    target.SetTarget(0, 0, 0, 1 , 0, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_17");
    }
    //目标18
    target.SetTarget(0, 0, 0, 1 , 0.8, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_18");
    }
    //目标19
    target.SetTarget(0, 0, 0, 1 , 1.6, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_19");
    }
    //目标20
    target.SetTarget(0, 0, 0, 1 , 2.4, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_20");
    }
    //目标21
    target.SetTarget(0, 0, 0, 1 , 2.4, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_21");
    }
    //目标22
    target.SetTarget(0, 0, 0, 1 , 2.4, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_22");
    }
    //目标23
    target.SetTarget(0, 0, 0, 1 , 2.4, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_23");
    }
    //目标24
    target.SetTarget(0, 0, 0, 1 , 1.6, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_24");
    }
    //目标25
    target.SetTarget(0, 0, 0, 1 , 0.8, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_25");
    }
    //目标26
    target.SetTarget(0, 0, 0, 1 , 0.8, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_26");
    }
    //目标27
    target.SetTarget(0, 0, 0, 1 , 0.8, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_27");
    }
    //目标28
    target.SetTarget(0, 0, 0, 1 , 1.6, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_28");
    }
    //目标29
    target.SetTarget( 0, 0, 0, 1 , 1.6, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_29");
    }
    //目标0
    target.SetTarget( 0, 0, 0, 1 , 0, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_29");
    }
    //目标0
    target.SetTarget( 0, 0, 0, 1 , 0, 0 , 0.5 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_29");
    }

    target.SetTarget(0, 0, 0, 1 , 0 , 0 , -0.1 );
    sm.process_event(drone::land{target});
    ros::Time time1 = ros::Time::now();
    while (sm.is("landing"_s) && ros::Time::now() - time1 < ros::Duration(5.0))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("Landing");
        
    }
    d.Set2Armed(false);
    d.openTFlisten(false);
    ROS_INFO("Locked!");
    sm.process_event(drone::To_lock{});
    return 0;
    }
    else if(mission_num == 2)
    {
            rate.sleep();
    drone::TFtarget target;
    
    ros::spinOnce();
    sm.process_event(drone::release{});
    //起飞到巡航高度1m
    for( double i = 1 ; i <= 5*10 ; i++ )
    {
        target.SetTarget( 0 , 0 , 0 , i/50 * 1.7 );
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

    bool isfound = 0;
    cv::Point2f center = cv::Point2f(0,0);
    double cam_intrinsics[4] = { 465.8243857636045, 466.0320923301909, 325.3179858695828, 257.065056004352 };
    cv::Point3d pos_rec;
    cv::Point3d pos_abs;
    cv::Point3d drone_pos;
    //目标1
    target.SetTarget(0, 0, 0, 1 , 0.8, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_1");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标2
    target.SetTarget(0, 0, 0, 1 , 1.6, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_2");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标3
    target.SetTarget(0, 0, 0, 1 , 2.4, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_3");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标4
    target.SetTarget(0, 0, 0, 1 , 3.2, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_4");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标5
    target.SetTarget(0, 0, 0, 1 , 3.2, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_5");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标6
    target.SetTarget(0, 0, 0, 1 , 3.2, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_6");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标7
    target.SetTarget(0, 0, 0, 1 , 3.2, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_7");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标8
    target.SetTarget(0, 0, 0, 1 , 3.2, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_8");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标9
    target.SetTarget(0, 0, 0, 1 , 3.2, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_9");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标10
    target.SetTarget(0, 0, 0, 1 , 2.4, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_10");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标11
    target.SetTarget(0, 0, 0, 1 , 1.6, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_11");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标12
    target.SetTarget(0, 0, 0, 1 , 0.8, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_12");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标13
    target.SetTarget(0, 0, 0, 1 , 0, -4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_13");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标14
    target.SetTarget(0, 0, 0, 1 , 0, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_14");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标15
    target.SetTarget(0, 0, 0, 1 , 0, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_15");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标16
    target.SetTarget(0, 0, 0, 1 , 0, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_16");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标17
    target.SetTarget(0, 0, 0, 1 , 0, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_17");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标18
    target.SetTarget(0, 0, 0, 1 , 0.8, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_18");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标19
    target.SetTarget(0, 0, 0, 1 , 1.6, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_19");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标20
    target.SetTarget(0, 0, 0, 1 , 2.4, -0.8 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_20");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标21
    target.SetTarget(0, 0, 0, 1 , 2.4, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_21");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标22
    target.SetTarget(0, 0, 0, 1 , 2.4, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_22");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标23
    target.SetTarget(0, 0, 0, 1 , 2.4, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_23");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标24
    target.SetTarget(0, 0, 0, 1 , 1.6, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_24");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标25
    target.SetTarget(0, 0, 0, 1 , 0.8, -3.2 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_25");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标26
    target.SetTarget(0, 0, 0, 1 , 0.8, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_26");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标27
    target.SetTarget(0, 0, 0, 1 , 0.8, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_27");
    }
        if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标28
    target.SetTarget(0, 0, 0, 1 , 1.6, -1.6 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_28");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标29
    target.SetTarget( 0, 0, 0, 1 , 1.6, -2.4 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_29");
    }
    if(isfound == 0)
    {
        isfound = get_red_position(center);
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() + 0.1 - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos);
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
            //下面开始显示LED灯
            // 两秒内一直发目标点
            ros::param::set("/LED",1);
            ros::Time T1 = ros::Time::now();
            while(ros::Time::now() - T1 < ros::Duration(2))
            {
                sm.process_event(drone::moveTo{target});
            }
            //sleep(2);
            ros::param::set("/LED",0);
            //下降到1m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 0.9 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1m");
            }
            //睡眠三秒
            ros::Time T2 = ros::Time::now();
            while(ros::Time::now() - T2 < ros::Duration(3))
            {
                sm.process_event(drone::moveTo{target});
            }
            //旋转舵机，掉落沙包
            ros::param::set("/Throw",1);
            sleep(1);
            //将目标位置发到小车
            ros::param::set("/px",int(pos_abs.x * 10));
            ros::param::set("/py",int(pos_abs.y * 10));
            //飞到1.8m
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red_1.8m");
            }
            ros::param::set("/Throw",0);

        }
    }
    //目标0
    target.SetTarget( 0, 0, 0, 1 , 0, 0 , 1.7 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_29");
    }
    //目标0
    target.SetTarget( 0, 0, 0, 1 , 0, 0 , 0.5 );
    sm.process_event(drone::moveTo{target});
    while (sm.is("moving"_s))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("moving_to_target_29");
    }

    target.SetTarget(0, 0, 0, 1 , 0 , 0 , -0.1 );
    sm.process_event(drone::land{target});
    ros::Time time1 = ros::Time::now();
    while (sm.is("landing"_s) && ros::Time::now() - time1 < ros::Duration(5.0))
    {
        sm.process_event(drone::tickOnce{});
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("Landing");
        
    }
    d.Set2Armed(false);
    d.openTFlisten(false);
    ROS_INFO("Locked!");
    sm.process_event(drone::To_lock{});
    return 0;
    }
}
