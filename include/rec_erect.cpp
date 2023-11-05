#include <iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
using namespace cv;
using namespace std;
//int main_erect(int argc, char* argv[])
//{
//	Mat Img = imread("D:/Desktop/code/zuanquan/img/ganzi.jpg");
//	Size rsize = Size(640, 480);
//	resize(Img, Img, rsize);
//	imshow("Source Image", Img);
//	waitKey(0);
//	return 0;
//}
//#include <opencv2/opencv.hpp>
//#include <iostream>
//
//int main()
//{
//    // 读取图像
//    cv::Mat image = cv::imread("D:/Desktop/code/zuanquan/img/ganzi.jpg");
//
//    if (image.empty())
//    {
//        std::cout << "无法加载图像" << std::endl;
//        return -1;
//    }
//    Size rsize = Size(640, 480);
//	resize(image, image, rsize);
//    // 转换图像到灰度
//    cv::Mat grayImage;
//    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
//    //cv::imshow("GRAY", grayImage);
//    //cv::waitKey(0);
//    // 进行边缘检测
//    cv::Mat edges;
//    cv::Canny(grayImage, edges, 50, 150);
//
//    // 执行霍夫直线变换
//    std::vector<cv::Vec2f> lines;
//    cv::HoughLines(edges, lines, 1, CV_PI / 180, 100);
//
//    // 在原始图像上绘制直线
//    for (size_t i = 0; i < lines.size(); i++)
//    {
//        //if (lines[i][0] > 500 && lines[i][1] > 0.5 && lines[i][1] < 3)
//        if (lines[i][0] > 300)
//        {
//            float rho = lines[i][0];
//            float theta = lines[i][1];
//            double a = cos(theta);
//            double b = sin(theta);
//            double x0 = a * rho;
//            double y0 = b * rho;
//            cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
//            cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
//            cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
//        }
//    }
//
//    // 显示结果
//    cv::imshow("Detected Lines", image);
//    cv::waitKey(0);
//
//    return 0;
//}

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

int main_erect()
{
    // 读取图像
    cv::Mat image = cv::imread("D:/Desktop/code/zuanquan/img/ganzi.jpg");
    cv::Mat image_source = image;
    Size rsize = Size(640, 480);
	resize(image, image, rsize);
    resize(image_source, image_source, rsize);
    cv::Mat imgHSV;
    cvtColor(image, imgHSV, COLOR_BGR2HSV);
    image = imgHSV;
    cv::inRange(image, cv::Scalar(0, 0, 59), cv::Scalar(67, 23, 188), image); //黄色
    if (image.empty())
    {
        std::cout << "无法加载图像" << std::endl;
        return -1;
    }

    // 转换图像到灰度
    cv::Mat grayImage;
    //cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    cv::imshow("hsv", image);
    // 进行边缘检测
    cv::Mat edges;
    //cv::Canny(grayImage, edges, 50, 150);
    //cv::imshow("canny", edges);
    // 执行概率霍夫直线变换
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(image, lines, 1, CV_PI / 180, 30, 200, 20);

    // 在原始图像上绘制直线段
    for (size_t i = 0; i < lines.size(); i++)
    {
        if (abs(lines[i][0] - lines[i][2]) < 20)
        {
            cv::Vec4i line = lines[i];
            cv::line(image_source, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }

    }

    // 显示结果
    cv::imshow("Detected Lines", image_source);
    cv::waitKey(0);

    return 0;
}
