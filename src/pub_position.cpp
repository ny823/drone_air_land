#include <iostream> //used for testing
#include "ros/ros.h"
#include "../include/state_machine.hpp"
#include <yaml-cpp/yaml.h>
#include "tf/transform_datatypes.h"
#include <math.h>
#include <tf/tf.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <serial/serial.h>

namespace sml = boost::sml;
using namespace sml;
using namespace std;


int main(int argc, char **argv)
{
    // 创建串口对象
    ros::init(argc, argv, "postion_pub");
    ros::NodeHandle nh;
    serial::Serial serial_port;
    drone::dependencies d;
    ros::Rate rate(10);
    d.n = nh;
    sml::sm<drone::icarus> sm{d, rate};
    //初始位置
    double x = 3;
    double y = 3;
    //拍照
    // cv::VideoCapture cap(0,cv::CAP_V4L2);
    // //cap = cv::VideoCapture(0, cv.CAP_V4L2);
	// cv::Mat image;  
	// cap >> image;
    // cv::imwrite("/home/drone/acfly_ws/src/linetracing/img/gao.jpg", image); 
    //通过串口发布当前位置信息
    //------------------------------------------------------------    
    try
    {
        // 打开串口
        serial_port.setPort("/dev/ttyTHS0");
        serial_port.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open the serial port: " << e.what());
        return -1;
    }
    if (serial_port.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized!");
        // 主循环，持续读取串口数据
        while (ros::ok())
        {
            // 读取串口数据
            // if (serial_port.available())
            // {
            //     std::string data = serial_port.read(serial_port.available());
            //     ROS_INFO_STREAM("Received data: " << data);
            // }

            // 在这里可以编写发送串口数据的逻辑
            // 使用serial_port.write()来发送数据
            x = int(3 - d.transform.getOrigin().y() * 10);
            y = int(3 + d.transform.getOrigin().x() * 10);
            std::vector<uint8_t> output_data = {0xBB, 0x01, 1, 0x00, 0x01, 0x01, 0xCC};
            //std::string output_data = [0xBB,0x01,0x02,0x01,0x01,0x01,0xCC];
            serial_port.write(output_data);
            // 延时一段时间
            ros::Duration(0.1).sleep();

            // 处理ROS回调函数
            ros::spinOnce();
        }
    }
    else
    {
        ROS_ERROR_STREAM("Serial port is not open!");
        return -1;
    }

    return 0;
    //------------------------------------------------------------
    // ros::init(argc, argv, "postion_pub");
    // ros::NodeHandle nh;
    // drone::dependencies d;
    // ros::Rate rate(10);
    // d.n = nh;
    // sml::sm<drone::icarus> sm{d, rate};
    // //初始位置
    // double x = 3.5;
    // double y = 3.5;
    // //获取无人机的实时位置
    // while(1)
    // {
    //     d.getTransform("camera_init", "base_link");
    //     // cout<<"odom_link_x: "<<d.transform.getOrigin().x()<<endl;
    //     // cout<<"odom_link_y: "<<d.transform.getOrigin().y()<<endl;
    //     // cout<<"odom_link_z: "<<d.transform.getOrigin().z()<<endl;
    //     //获取无人机的坐标
    //     x = int(3 - d.transform.getOrigin().y() * 10);
    //     y = int(3 + d.transform.getOrigin().x() * 10);
    //     cout<< x <<endl;
    //     cout<< y <<endl;
    //     cout<<"-------------------------------------------"<<endl;
    //     ros::Duration(1).sleep();
    // }

    // return 0;
}