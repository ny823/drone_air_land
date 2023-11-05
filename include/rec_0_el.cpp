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
#include <opencv2/calib3d/calib3d.hpp>
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "../include/color.hpp"//颜色提取
#include "../include/EDLib.h"
#include "../include/rec_o_l.hpp"
#include "../include/rec_line_new.hpp"
#include <zxing/LuminanceSource.h>
#include <zxing/common/Counted.h>
#include <zxing/Reader.h>
#include <zxing/ReaderException.h>
#include <zxing/Exception.h>
#include <zxing/aztec/AztecReader.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/DecodeHints.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>
#include <zxing/datamatrix/DataMatrixReader.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/pdf417/PDF417Reader.h>
#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/MatSource.h>
#include <zxing/oned/Code128Reader.h>

#include "../include/rec.hpp"


using namespace cv;
using namespace std;
using namespace message_filters;
using namespace rec;
cv::Mat left_image;
cv::Mat right_image;
/*
void image_callback(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img)
{
    cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::TYPE_8UC1);
    left_image = cv_ptr1->image;
    cv_bridge::CvImagePtr cv_ptr2 = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::TYPE_8UC1);
    right_image = cv_ptr2->image;
}
*/
//深度�????
//深度图和彩图获取
cv::Mat image_dep;
cv::Mat img;
cv::Mat img_p;
//圆两边的深度
int ll = 0;
double dep1;
double dep2;
double dep_gan;
double dep1_min = 4000; 
double dep2_min = 4000;
double dep_gan_min = 4000;
double x_p;
double y_p;
double x_m;
double y_m;
double angle1;
int x_middle,y_middle;
//两个话题同步获取


// 识别椭圆
void image_callback(const sensor_msgs::ImageConstPtr &dep_image,const sensor_msgs::ImageConstPtr &color_image)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(dep_image, "16UC1");
    image_dep = cv_ptr->image;
    cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(color_image, "bgr8");
    img = cv_ptr1->image;
    Mat imggray;
	Mat testImg, cdst;
	cv::cvtColor(img, imggray, cv::COLOR_BGR2GRAY);
	// cv::inRange(imghsv, cv::Scalar(35, 43, 46), cv::Scalar(77, 255, 255), testImg); //绿色
    cv::Mat rep_img;
    mEllipse ellipses;
    bool isappear;
    isappear = rec_ellipses_o(imggray,rep_img,ellipses);
    if(isappear)
    {
    cv::Point3d normal;
    double cam_intrinsics[4] = {388.3740805921295, 388.4762415323952, 327.6361232087904, 242.92972911792748};
    std::vector<cv::Point3d> point_raw = ransac(ellipses,image_dep,normal,cam_intrinsics);
    cv::Point3d position = cirPose(point_raw,ellipses,cam_intrinsics);
    cout << "AAAAA" << endl;//圆的位姿
    cout << normal.x << endl;
    cout << normal.y << endl;
    cout << normal.z << endl;
    cout << atan(normal.z/normal.x)*180/3.14  << endl;
    cout << "BBBBBBB" << endl;//圆形坐标
    cout << position.x << endl;
    cout << position.y << endl;
    cout << position.z << endl;
    cout << "CCCCCCC" << endl;
    }
    // if(!(rep_img.empty()))
    // {
    // cv::imwrite("/home/drone/acfly_ws/src/linetracing/img/00.jpg", cv_ptr1->image);    
    // cout << "ok" << endl;
    // }


	// cv::Mat elementRect;
	// elementRect = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
	// cv::morphologyEx(testImg, testImg, cv::MORPH_CLOSE, elementRect);
	//cv::inRange(imghsv, cv::Scalar(29, 50, 0), cv::Scalar(46, 255, 255), testImg); //黄色
	//Call ED constructor


}
//识别杆子
// void image_callback(const sensor_msgs::ImageConstPtr &dep_image,const sensor_msgs::ImageConstPtr &color_image)
// {
//     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(dep_image, "16UC1");
//     image_dep = cv_ptr->image;
//     cv_bridge::CvImagePtr cv_ptr1 = cv_bridge::toCvCopy(color_image, "bgr8");
//     img = cv_ptr1->image;
//     Mat rep_img, rep_img1;
// 	std::vector<std::pair<LS, LS>> pole_recs_ = detect_pole(img, rep_img, CV_8UC3);
// 	std::vector<cv::Point3d> screened_pole;
// 	std::vector<cv::Point3d> depth_vec;
//     for (int i = 0; i < pole_recs_.size(); i++)
//     {
// 	    depth_vec = get_point(pole_recs_, image_dep);
// 	    double cam_intrinsics[4] = { 388.3740805921295, 388.4762415323952, 327.6361232087904, 242.92972911792748 };
// 	    screened_pole.push_back(ransac_double(depth_vec, cam_intrinsics, 0.05, 12));
//     }
//     cout << screened_pole.size() << endl;
// 	std::vector<cv::Point3d> true_pole_;
// 	true_pole_ = true_pole(screened_pole, cv::Point3d(-1000, 1, 1500), cv::Point3d(1000, 1, 2500));
//     // if(!(rep_img.empty()))
//     // {
//     // cv::imwrite("/home/drone/acfly_ws/src/linetracing/img/00.jpg", cv_ptr1->image);    
//     // cout << "ok" << endl;
//     // }
//     cout << "AAAAA" << endl;
//     // cout << normal.x << endl;
//     // cout << normal.y << endl;
//     // cout << normal.z << endl;
//     cout << "BBBBBBB" << endl;
// 	// cv::Mat elementRect;
// 	// elementRect = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
// 	// cv::morphologyEx(testImg, testImg, cv::MORPH_CLOSE, elementRect);
// 	//cv::inRange(imghsv, cv::Scalar(29, 50, 0), cv::Scalar(46, 255, 255), testImg); //黄色
// 	//Call ED constructor


// }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dep_img_get");
    ros::NodeHandle nh;
    //message_filters::Subscriber sub1 = nh.subscribe("/camera/depth/image_rect_raw", 1000, image_callback);
    // ros::Subscriber sub2 = nh.subscribe("/camera/color/image_raw", 1000, image_callback1);
    // message_filters::Subscriber<sensor_msgs::Image> sub_color_image(nh, "/camera/aligned_depth_to_color/image_raw", 2000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> sub_color_image(nh, "/camera/aligned_depth_to_color/image_raw", 2000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> sub_right_image(nh, "/camera/color/image_raw", 2000, ros::TransportHints().tcpNoDelay());
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), sub_color_image, sub_right_image);
    //指定一个回调函数，就可以实现两个话题数据的同步获取
    sync.registerCallback(boost::bind(&image_callback, _1, _2));
    ros::Rate rate(10);
    // for( int o = 0 ; o < 10;o++)
    // {
    //     ros::spinOnce();
    // }
    ros::spin();
    // Mat src = imread("/home/drone/acfly_ws/src/linetracing/img/qrcode1.png");
    // cv::imwrite("/home/drone/acfly_ws/src/linetracing/img/qrcode11.jpg", src);
    return 0;
}
