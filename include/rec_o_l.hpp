#include <iostream> //used for testing
#include "ros/ros.h"
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

//识别椭圆
void rec_o(cv::Mat& testImg,cv::Mat& color_img,cv::Mat& image_dep,double& dep1,double& dep2,double& dep1_min, double& dep2_min, double& dd)
{
  ED testED = ED(testImg, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm
  EDCircles testEDCircles = EDCircles(testImg);
	Mat circleImg = testEDCircles.drawResult(false, ImageStyle::ELLIPSES);
  vector<mEllipse> ellipses = testEDCircles.getEllipses();
  // cout << ellipses.size() << endl;
  dep1 = 0;
  dep2 = 0;
  dep1_min = 5000; 
  dep2_min = 5000;
  cout << ellipses.size() << endl;
  if(ellipses.size()>0)
  {
    cout << "find" << endl;
    cout << ellipses[0].center.x << endl;
	  cout << ellipses[0].center.y << endl;
	  cout << ellipses[0].axes.width<< endl;
	  cout << ellipses[0].axes.height << endl;
    //cout << (int(ellipses[0].center.x) - int(ellipses[0].axes.width)) <<endl;  
    for(int ii = int(ellipses[0].center.x) - int(ellipses[0].axes.width) - 10;ii < int(ellipses[0].center.x) - int(ellipses[0].axes.width ) + 10;ii++)
    {
      dep1 = image_dep.at<ushort>(int(ellipses[0].center.y), ii );
      if(dep1 < dep1_min && dep1 > 1000)
      {
        dep1_min = dep1;
      } 
    }
    dep1 = dep1_min;
    for(int jj = int(ellipses[0].center.x) + int(ellipses[0].axes.width) - 10;jj < int(ellipses[0].center.x) + int(ellipses[0].axes.width ) + 10;jj++)
    {
      dep2 = image_dep.at<ushort>(int(ellipses[0].center.y), jj );
      if(dep2 < dep2_min && dep2 > 1000)
      {
        dep2_min = dep2;
      }      
    }
    dep2 = dep2_min; 
    dd = (ellipses[0].center.x - ellipses[0].axes.width - 320) * dep1 / 388;
    // angle1 = acos((dep1 - dep2) / 1100);
    // x_p = ((dd + 550 / sin(angle1) - dep1 / tan(angle1)) * sin(angle1)) * cos(angle1);
    // y_p = ((dd + 550 / sin(angle1) - dep1 / tan(angle1)) * sin(angle1)) * sin(angle1);
    // if(dep1 > dep2)
    // {
    //     x_m = (dep1 + dep2) / 2 + 700 * cos(angle1);
    //     y_m = dd + 550 * sin(angle1) + 700 * sin(angle1);
    // }
    // else if(dep1 <= dep2)
    // {
    //     x_m = (dep1 + dep2) / 2 - 700 * cos(angle1);
    //     y_m = dd + 550 * sin(angle1) - 700 * sin(angle1);
    // }
    cout << dep1 <<endl;
    cout << dep2 <<endl;
    cout << "--------------------------" << endl;
  }
}
//识别�?
void rec_l(cv::Mat edges,cv::Mat& image_dep,double& dep_gan,double& dep_gan_min,int& x_middle,int& y_middle)
{
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 300, 60);
    
    for (size_t i = 0; i < lines.size(); i++)
    {
        if (abs(lines[i][0] - lines[i][2]) < 20 && lines[i][0] >50 && lines[i][0] <440)
        {

            x_middle = (lines[i][0] + lines[i][2]) / 2;
            y_middle = (lines[i][1] + lines[i][3]) / 2;
            break;            
        }
    }
        //在原始图像上绘制直线�?
    // for (size_t i = 0; i < lines.size(); i++)
    // {
    //     if (abs(lines[i][0] - lines[i][2]) < 20)
    //     {
    //         cv::Vec4i line = lines[i];
    //         cv::line(image_source, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    //         break;
    //     }

    // }
    dep_gan = 0;
    dep_gan_min = 4000;
    for(int ii = x_middle - 10;ii < x_middle + 10 ;ii++)
    {
        dep_gan = image_dep.at<ushort>(y_middle , ii );
        if(dep_gan < dep_gan_min && dep_gan > 700)
        {
            dep_gan_min = dep_gan;
        }   
    }
    dep_gan = dep_gan_min;
    //dep_gan = image_dep.at<ushort>(y_middle,x_middle);
    if(dep_gan > 0 && dep_gan < 3000)
    {
        cout << "------------------------" << endl;
        cout << x_middle << endl;
        cout << y_middle << endl;
        cout << dep_gan << endl;      
    }
    cout << "------------------------" << endl;
}

void rec_rectangle()
{
  
}
