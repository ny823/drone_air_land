#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace rec_red
{
    bool get_red_position(cv::Point2f center& center)
    {
        cv::VideoCapture cap(0);
	    //cap.set(CAP_PROP_AUTOFOCUS, 1);
	    //cap.set(CAP_PROP_FOCUS, 100);
	    cap.set(CAP_PROP_FRAME_WIDTH, 640);
	    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
	    //cap.set(CAP_PROP_FRAME_WIDTH, 640);//宽度
	    //cap.set(CAP_PROP_FRAME_HEIGHT, 480);//高度
	    cap.set(CAP_PROP_FPS, 30);//帧率 帧/秒
	    cap.set(CAP_PROP_BRIGHTNESS, 0);//亮度 1
	    cap.set(CAP_PROP_CONTRAST, 32);//对比度 40
	    cap.set(CAP_PROP_SATURATION, 60);//饱和度 50
	    cap.set(CAP_PROP_HUE, 0);//色调 50
	    cap.set(CAP_PROP_EXPOSURE, -8);//曝光 50
	    cap.set(CAP_PROP_WB_TEMPERATURE, 5533);//色温
	    cap.set(CAP_PROP_GAMMA, 100);//gamma
        cv::Mat image;
        cap >> image;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        // 颜色范围定义（红色）
        cv::Mat imgThresholded1, imgThresholded2;
        Mat imgThresholded;
        cv::inRange(hsv, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), imgThresholded1); //红色
        cv::inRange(hsv, cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255), imgThresholded2);
        add(imgThresholded1, imgThresholded2, imgThresholded);
        //cv::Scalar lowerGreen(40, 50, 50); // 最小颜色值（H, S, V）
        //cv::Scalar upperGreen(80, 255, 255); // 最大颜色值（H, S, V）

        // 创建掩膜（mask）
        cv::Mat mask = imgThresholded;
        //cv::inRange(hsv, lowerGreen, upperGreen, mask);

        // 二值化
        cv::Mat binary;
        cv::threshold(mask, binary, 0, 255, cv::THRESH_BINARY);

        // 形态学操作（可选）
        cv::Mat morphology;
        cv::dilate(binary, morphology, cv::Mat());
        cv::erode(morphology, morphology, cv::Mat());

        // 轮廓检测
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(morphology, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0)
        {
            std::cout << "not found" << std::endl;
            return false;
        }
        for (const auto& contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > 100) {
                // 计算轮廓的几何矩
                cv::Moments moments = cv::moments(contour);

                // 计算轮廓的中心坐标
                center(
                    static_cast<float>(moments.m10 / moments.m00),
                    static_cast<float>(moments.m01 / moments.m00)
                );

                // 绘制轮廓
                //cv::drawContours(image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 2);

                //// 在图像上绘制中心点
                //cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);

                // 在控制台输出中心坐标
                std::cout << "Contour center: (" << center.x << ", " << center.y << ")" << std::endl;
            }
        }
        return true;
    }
    //获取相对位置
    cv::Point3d get_relative_pose(cv::Point2d point, double depth, double cam_intrinsics[4])
	{
		double x = (point.x - cam_intrinsics[2]) * depth / cam_intrinsics[0];
		double y = (point.y - cam_intrinsics[3]) * depth / cam_intrinsics[1];
		cv::Point3d position = cv::Point3d(x, y, depth);//此处是相机的xyz
		cout << "x:" << position.x << endl;
		cout << "y:" << position.y << endl;
		cout << "z:" << position.z << endl;
		return position;
	}
    //获取目标绝对位置
    //根据相对位置获取绝对位置
	/*
	input: 相对位置 position 飞机自身的绝对位置：drone_pos 
	output: 绝对位置 abs_position
	position.x,position.y,position.z
	*/
	cv::Point3d get_abs_pose(cv::Point3d position, cv::Point3d drone_pos)
	{
		cv::Point3d abs_position;
		abs_position.x = drone_pos.x + position.z;
		abs_position.y = drone_pos.y - position.x;
		abs_position.z = drone_pos.z - position.y;//固定1.8m
        
		return abs_position;
	}

}