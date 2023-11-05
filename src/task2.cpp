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

#include "../include/rec_red.hpp"


namespace sml = boost::sml;
using namespace sml;
using namespace std;
using namespace rec_red;

// 

// void state_cb(const mavros_msgs::State::ConstPtr &msg)
// {
//     try
//     {
//         // 打开串口
//         serial_port.setPort("/dev/ttyTHS0");
//         serial_port.setBaudrate(115200);
//         serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
//         serial_port.setTimeout(timeout);
//         serial_port.open();
//     }
//     catch (serial::IOException& e)
//     {
//         ROS_ERROR_STREAM("Unable to open the serial port: " << e.what());
//     }
//     if (serial_port.isOpen())
//     {
//         ROS_INFO_STREAM("Serial port initialized!");
//         // 主循环，持续读取串口数据
//             // 读取串口数据
//             // if (serial_port.available())
//             // {
//             //     std::string data = serial_port.read(serial_port.available());
//             //     ROS_INFO_STREAM("Received data: " << data);
//             // }

//             // 在这里可以编写发送串口数据的逻辑
//             // 使用serial_port.write()来发送数据
//             x1 = int(3 - d.transform.getOrigin().y() * 10);
//             y1 = int(3 + d.transform.getOrigin().x() * 10);
//             std::vector<uint8_t> output_data = {0xBB, 0x01, 0, 0x00, x1, y1, 0xCC};
//             //std::string output_data = [0xBB,0x01,0x02,0x01,0x01,0x01,0xCC];
//             serial_port.write(output_data);
//             // 延时一段时间
//             //ros::Duration(0.1).sleep();

//             // 处理ROS回调函数
//             //ros::spinOnce();
//     }
//     else
//     {
//         ROS_ERROR_STREAM("Serial port is not open!");
//     }

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control_test");

    ros::NodeHandle nh;
    drone::dependencies d;
    ros::Rate rate(10);
    d.n = nh;
    sml::sm<drone::icarus> sm{d, rate};
    // serial::Serial serial_port;
    // try
    // {
    //     // 打开串口
    //     serial_port.setPort("/dev/ttyTHS0");
    //     serial_port.setBaudrate(115200);
    //     serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    //     serial_port.setTimeout(timeout);
    //     serial_port.open();
    // }
    // catch (serial::IOException& e)
    // {
    //     ROS_ERROR_STREAM("Unable to open the serial port: " << e.what());
    //     return -1;
    // }
    // if (serial_port.isOpen())
    // {
    //     ROS_INFO_STREAM("Serial port initialized!");
    //     // 主循环，持续读取串口数据
    //     while (ros::ok() && data != 1)
    //     {
    //         // 读取串口数据
    //         if (serial_port.available())
    //         {
    //             data = int(serial_port.read(serial_port.available())) & 010;
    //         }
    //         // 延时一段时间
            
    //         ros::Duration(0.1).sleep();
    //     }
    // }
    // else
    // {
    //     ROS_ERROR_STREAM("Serial port is not open!");
    //     return -1;
    // }
    // ros::spin();
    // sleep(5);
    // ROS_INFO("5s left!!");
    // sleep(5);
    // ROS_INFO("Mission Start!");
   
    rate.sleep();
    drone::TFtarget target;
    

    // geometry_msgs::Quaternion rotate;
    // tf::Quaternion tf_rotate; 

    //延时10秒起飞
    // sleep(10);
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


    bool isfound = 0;
    cv::Point2f& center;
    double cam_intrinsics[4] = { 465.8243857636045, 466.0320923301909, 325.3179858695828, 257.065056004352 };
    cv::Point3d pos_rec;
    cv::Point3d pos_abs;
    cv::Point3d drone_pos;
    cv::Point3d position = cv::Point3d(x, y, depth);
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
        isfound = get_red_position(cv::Point2f& center)
        if(isfound == 1)
        {		
		    pos_rec = get_relative_pose(center, d.transform.getOrigin().z() - 0.22, cam_intrinsics);
            drone_pos = cv::Point3d(d.transform.getOrigin().x(),d.transform.getOrigin().y(),d.transform.getOrigin().z() + 0.1);
            pos_abs = get_abs_pose(pos_rec, drone_pos)
            target.SetTarget(0, 0, 0, 1 , pos_abs.x, pos_abs.y , 1.7 );
            sm.process_event(drone::moveTo{target});
            while (sm.is("moving"_s))
            {
                sm.process_event(drone::tickOnce{});
                rate.sleep();
                ros::spinOnce();
                ROS_INFO("moving_to_red");
            }
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
