#include "EDLib.h"
#include <iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<math.h>
//#include <Eigen/Core>
//#include <Eigen/Dense>
//#include <nav_msgs/Odometry.h>
//#include <sensor_msgs/Image.h>
//#include <geometry_msgs/Twist.h>

//ransac筛点的参数
double ransc_threshold = 15;
double inlier_ratio = 0.3;
double maximumIterations = 30;
int minimum_area = 5000;
int num_eli = 500;

//识别的深度范围
int dep_max = 3000;//ransc中使用
int dep_min = 100;

//EDLINE用
double _line_error = 1.0;
double _max_distance_between_two_lines = 6.0;
double _max_error = 1.300001;
int _min_line_len = 30;

//识别杆子用
double minDis = 1, maxDis = 20;                                           // 判断两直线是否是一对
double kPole = 5;                                                    // 杆子斜率
double LS_k = 0.04;                                                     // 判断两线段是否为同一直线；
double minDisDiff = 5;                                               // 判断两线段是否为同一直线；
double minPole = 30000;                                                  // 杆子长度最小值
double ratio = 10;                                                    // 两根杆子的长度比值限制
double is_Line = 3;                                                  // 两根杆子的最小误差
double aspect_ratio = 20;                                             // 识别的长方形的长宽比限制


using namespace std;
using namespace cv;
namespace rec
{
	//椭圆识别
/*
	input: 灰度图像
	output:mEllipse ellipses
	mEllipse {
	cv::Point2d center;
	cv::Size axes;
	double theta;旋转角度顺时针为正，逆时针为负
	若要控制识别到的圆的大小，则设置该变量的值#define CIRCLE_MIN_LINE_LEN 6
};
*/
	mEllipse rec_ellipses(cv::Mat& testImg)//input with grayimg可以是HSV处理的
	{
		ED testED = ED(testImg, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm
		EDCircles testEDCircles = EDCircles(testED);
		vector<mEllipse> ellipses = testEDCircles.getEllipses();
		//获取识别到的最大椭圆
		int noellipse = ellipses.capacity();
		int find_max_ellipses = 0;
		if (noellipse != 0)
		{
			for (int i = 1; i < noellipse; i++)
			{
				if (ellipses[find_max_ellipses].axes.area() < ellipses[i].axes.area())
					find_max_ellipses = i;
			}
			cout << "find ellipses, ellipses number == " + noellipse << endl;
		}
		if (noellipse == 0)
		{
			cout << "not find ellipses" << endl;
		}
		//显示图像
		//Mat circleImg = testEDCircles.drawResult(true, ImageStyle::ELLIPSES);//显示椭圆,true在原图上画，false在空白图上面画
		//imshow("Color Circle", circleImg);
		//waitKey();
		//cout << find_max_ellipses << endl;
		return ellipses[find_max_ellipses];
	}
	mEllipse rec_ellipses_free(cv::Mat& testImg)
	{
		EDPF testEDPF = EDPF(testImg);
		EDCircles testEDCircles = EDCircles(testEDPF);
		vector<mEllipse> ellipses = testEDCircles.getEllipses();
		//获取识别到的最大椭圆
		int noellipse = ellipses.capacity();
		int find_max_ellipses = 0;
		if (noellipse != 0)
		{
			for (int i = 1; i < noellipse; i++)
			{
				if (ellipses[find_max_ellipses].axes.area() < ellipses[i].axes.area())
					find_max_ellipses = i;
			}
			cout << "find ellipses, ellipses number == " + noellipse << endl;
		}
		if (noellipse == 0)
		{
			cout << "not find ellipses" << endl;
		}
		//显示图像
		//Mat circleImg = testEDCircles.drawResult(true, ImageStyle::ELLIPSES);//显示椭圆,true在原图上画，false在空白图上面画
		//imshow("Color Circle", circleImg);
		//waitKey();
		return ellipses[find_max_ellipses];
	}
	//圆形识别
	/*
		input: 灰度图像
		output:vector<mCircle> circles
		mCircle {
		cv::Point2d center;
		double r;
	};
	*/
	mCircle rec_o(cv::Mat& testImg)//input with grayimg可以是HSV处理的
	{
		ED testED = ED(testImg, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm
		EDCircles testEDCircles = EDCircles(testED);
		vector<mCircle> circles = testEDCircles.getCircles();
		//获取识别到的最大圆
		int nocircle = circles.capacity();
		int find_max_circle = 0;
		if (nocircle != 0)
		{
			for (int i = 1; i < nocircle; i++)
			{
				if (circles[find_max_circle].r < circles[i].r)
					find_max_circle = i;
			}
			cout << "find ellipses, ellipses number == " + nocircle << endl;
		}
		if (nocircle == 0)
		{
			cout << "not find circles" << endl;
		}
		//显示图像
		//Mat circleImg = testEDCircles.drawResult(true, ImageStyle::CIRCLES);//显示椭圆,true在原图上画，false在空白图上面画
		//imshow("Color Circle", circleImg);
		//waitKey();
		return circles[find_max_circle];
	}
	mCircle rec_o_free(cv::Mat& testImg)
	{
		EDPF testEDPF = EDPF(testImg);
		EDCircles testEDCircles = EDCircles(testEDPF);
		vector<mCircle> circles = testEDCircles.getCircles();
		//获取识别到的最大圆
		int nocircle = circles.capacity();
		int find_max_circle = 0;
		if (nocircle != 0)
		{
			for (int i = 1; i < nocircle; i++)
			{
				if (circles[find_max_circle].r < circles[i].r)
					find_max_circle = i;
			}
			cout << "find ellipses, ellipses number == " + nocircle << endl;
		}
		if (nocircle == 0)
		{
			cout << "not find circles" << endl;
		}
		return circles[find_max_circle];
	}
	//椭圆和圆形识别
	/*
		input: 灰度图像
		output:mEllipse ellipses
		mCircle {
		cv::Point2d center;
		double r;
	};
	*/
	mEllipse rec_ellipses_o(cv::Mat& testImg, cv::Mat& rep_img)//input with grayimg可以是HSV处理的
	{
		
		ED testED = ED(testImg, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm
		EDCircles testEDCircles = EDCircles(testED);
		vector<mEllipse> ellipses = testEDCircles.getEllipses();
		vector<mCircle> circles = testEDCircles.getCircles();
		//获取识别到的最大椭圆
		int noellipse = ellipses.capacity();
		int nocircle = circles.capacity();
		int find_max_circle = 0;
		int find_max_ellipses = 0;
		if (noellipse != 0)
		{
			for (int i = 1; i < noellipse; i++)
			{
				if (ellipses[find_max_ellipses].axes.area() < ellipses[i].axes.area())
					find_max_ellipses = i;
			}
			//cout << "find ellipses, ellipses number == " + noellipse << endl;
		}
		if (nocircle != 0)
		{
			for (int i = 1; i < nocircle; i++)
			{
				if (circles[find_max_circle].r < circles[i].r)
					find_max_circle = i;
			}
			//cout << "find ellipses, ellipses number == " + nocircle << endl;
		}
		cout<< "---------------------------" << endl;
		cout << "nocircle:" << nocircle << "\tnoellipse:" << noellipse << endl;
		cout << ellipses[find_max_ellipses].center.x << endl;
		cout << ellipses[find_max_ellipses].center.y << endl;
		cout << ellipses[find_max_ellipses].axes.width << endl;
		cout << ellipses[find_max_ellipses].axes.height << endl;

		mEllipse ellipse_raw = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
		bool isappeared;
		if (noellipse != 0 && nocircle != 0)
		{
			if (pow(circles[find_max_circle].r, 2) > ellipses[find_max_ellipses].axes.area())
			{
				if (pow(circles[find_max_circle].r, 2) > minimum_area)
				{
					cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
					ellipse_raw = mEllipse(circles[find_max_circle].center, r, 0);
					cv::circle(rep_img, circles[find_max_circle].center, (int)circles[find_max_circle].r, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
					isappeared = true;
				}
				else
				{
					isappeared = false;//欧克
				}
			}
			else
			{
				if (ellipses[find_max_ellipses].axes.area() > minimum_area)
				{
					double degree = (ellipses[find_max_ellipses].theta * 180) / PI; // convert radian to degree (opencv's ellipse function works with degree)
					cv::ellipse(rep_img, ellipses[find_max_ellipses].center, ellipses[find_max_ellipses].axes, degree, 0.0, 360.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
					isappeared = true;
					ellipse_raw = ellipses[find_max_ellipses];
				}
				else
				{
					isappeared = false;
				}
			}
		}
		else if (nocircle != 0 && noellipse == 0)
		{
			if (pow(circles[find_max_circle].r, 2) > minimum_area)
			{
				cv::circle(rep_img, circles[find_max_circle].center, (int)circles[find_max_circle].r, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
				cv::Size r = cv::Size((int)circles[find_max_circle].r, (int)circles[find_max_circle].r);
				ellipse_raw = mEllipse(circles[find_max_circle].center, r, 0);
				isappeared = true;
			}
			else
			{
				isappeared = false;
			}
		}
		else if (nocircle == 0 && noellipse != 0)
		{
			if (ellipses[find_max_ellipses].axes.area() > minimum_area)
			{
				isappeared = true;
				ellipse_raw = ellipses[find_max_ellipses];
				double degree = (ellipses[find_max_ellipses].theta * 180) / PI; // convert radian to degree (opencv's ellipse function works with degree)
				cv::ellipse(rep_img, ellipses[find_max_ellipses].center, ellipses[find_max_ellipses].axes, degree, 0.0, 360.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
			}
			else
			{
				isappeared = false;
			}
		}
		else
		{
			isappeared = false;
			ellipse_raw = mEllipse(cv::Point2d(0, 0), cv::Size(0, 0), 0);
		}
		cout << "get_eli:" << isappeared << endl;
		return ellipse_raw;
		//显示图像
		//Mat circleImg = testEDCircles.drawResult(true, ImageStyle::ELLIPSES);//显示椭圆,true在原图上画，false在空白图上面画
		//imshow("Color Circle", circleImg);
		//waitKey();
		//cout << find_max_ellipses << endl;
		//return ellipses[find_max_ellipses];
	}
	//ransac筛点
	/*
		input: ellipse_raw
		output:std::vector<cv::Point3d>筛选出来的点
	*/
	std::vector<cv::Point3d> ransac(mEllipse ellipse_raw,cv::Mat& dep_img, cv::Point3d& normal, double cam_intrinsics[4])
	{
		// 得到初始点云
		// x = x₀ + a * cos(θ) * cos(t) - b * sin(θ) * sin(t)
		// y = y₀ + a * sin(θ) * cos(t) + b * cos(θ) * sin(t)
		//sensor_msgs::Image depth_msg_ = depth_msg;
		//cv::Point3d position, normal;//圆的位姿和圆心坐标
		std::vector<cv::Point3d> point_raw;
		double theta = ellipse_raw.theta;
		cv::Point2d center = ellipse_raw.center;
		cv::Size axes = ellipse_raw.axes;
		float step = 2 * PI / num_eli;
		// 形成椭圆三维点云
		for (int i = 0; i < num_eli; i++)
		{
			float t = i * step;
			int u = center.x + axes.width * cos(theta) * cos(t) - axes.height * sin(theta) * sin(t);
			int v = center.y + axes.width * sin(theta) * cos(t) + axes.height * cos(theta) * sin(t);
			for (int u_x = -4; u_x < 5; u_x++)
			{
				for (int v_y = -4; v_y < 5; v_y++)
				{
					int u_ = u + u_x;
					int v_ = v + v_y;
					double depth = dep_img.at<ushort>(v_, u_);
					if (depth > dep_min && depth < dep_max)
					{
						double x = (u_ - cam_intrinsics[2]) * depth / cam_intrinsics[0];
						double y = (v_ - cam_intrinsics[3]) * depth / cam_intrinsics[1];
						point_raw.push_back(cv::Point3d(x, y, depth));
						// 清零标记已记录
						dep_img.at<ushort>(v_, u_) = 0;
					}
					else if (depth != 0)
					{
						dep_img.at<ushort>(v_, u_) = 0;
						// depth_msg_.data[v_ * width * 2 + u_ * 2] = 0;
						// depth_msg_.data[v_ * width * 2 + u_ * 2 + 1] = 0;
						//continue;
					}
				}
			}
		}
		double bestPlane[4];
		// 进行ransac筛点
		int iterationNumber = 0;
		std::vector<int> inliers, bestInliers;
		cout << "Point_raw size:" << point_raw.size() << endl;
		while (iterationNumber++ < maximumIterations && point_raw.size() > 3)
		{
			double aver_dis = 0;
			cout << "iterationNumber:" << iterationNumber << endl;
			int sample[3];
			// 1. Select a minimal sample, i.e., in this case, 3 random points.
			for (size_t sampleIdx = 0; sampleIdx < 3; sampleIdx++)
			{

				// Generate a random index between [0, pointNumber]
				sample[sampleIdx] =
					round((point_raw.size() - 1) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX));

				// If the first point is selected we don't have to check if
				// that particular index had already been selected.
				if (sampleIdx == 0)
					continue;
				if (sampleIdx == 1 &&
					sample[0] == sample[1])
				{
					sampleIdx--;
					continue;
				}
				if (sampleIdx == 2 && (sample[1] == sample[2] ||
					sample[0] == sample[2]))
				{
					sampleIdx--;
					continue;
				}
			}
			cv::Point3d p1 = point_raw[sample[0]];
			cv::Point3d p2 = point_raw[sample[1]];
			cv::Point3d p3 = point_raw[sample[2]];
			// These two vectors are in the plane
			cv::Point3d v1 = p3 - p1;
			cv::Point3d v2 = p2 - p1;

			// Cross product
			cv::Point3d cp = v1.cross(v2);
			cp = cp / cv::norm(cp);
			double a = cp.x;
			double b = cp.y;
			double c = cp.z;
			double d = -cp.ddot(p3);

			// 3. Count the number of inliers, i.e., the points closer than the threshold.
			inliers.clear();
			for (size_t pointIdx = 0; pointIdx < point_raw.size(); pointIdx++)
			{
				cv::Point3d point = point_raw[pointIdx];
				const double distance = abs(a * point.x + b * point.y + c * point.z + d);
				if (distance < ransc_threshold)
				{
					aver_dis = aver_dis + distance;
					inliers.push_back(pointIdx);
				}
			}
			if (inliers.size() != 0)
			{
				aver_dis = aver_dis / inliers.size();
				cout << "inliers.size:" << inliers.size() << "\taver_dis" << aver_dis << endl;
			}
			else
			{
				aver_dis = 0;
				cout << "inliers.size:" << 0 << "\taver_dis" << 0 << endl;
			}
			cout << "size" << point_raw.size() << endl;
			if (inliers.size() == point_raw.size() && aver_dis < ransc_threshold / 3)
			{
				bestInliers.swap(inliers);
				inliers.clear();
				inliers.resize(0);

				bestPlane[0] = a;
				bestPlane[1] = b;
				bestPlane[2] = c;
				bestPlane[3] = d;
				break;
			}
			// 4. Store the inlier number and the plane parameters if it is better than the previous best.
			if (inliers.size() > inlier_ratio * point_raw.size() && inliers.size() > bestInliers.size())
			{
				bestInliers.swap(inliers);
				inliers.clear();
				inliers.resize(0);

				bestPlane[0] = a;
				bestPlane[1] = b;
				bestPlane[2] = c;
				bestPlane[3] = d;
			}
			
		}
		std::vector<cv::Point3d> point_circle;
		cout << bestInliers.size() << endl;
		cout << inlier_ratio * point_raw.size() << endl;
		if (bestInliers.size() > inlier_ratio * point_raw.size())
		{
			for (int i = 0; i < bestInliers.size(); i++)
			{
				point_circle.push_back(point_raw[bestInliers[i]]);
			}
			normal = cv::Point3d(bestPlane[0], bestPlane[1], bestPlane[2]);
			cout << "normal:" << normal.x << "\t"
				<< normal.y << "\t"
				<< normal.z << endl;
		}
		else
		{
			normal = cv::Point3d(0, 0, 0);
			cout << "normal:"
				<< "0\t0\t0" << endl;
		}
		return point_circle;
	}

	//获取圆心的相对位置
	cv::Point3d cirPose(std::vector<cv::Point3d>& circle_point3d, mEllipse ellipse_raw, double cam_intrinsics[4])
	{
		// 选取3d点云中最上方的一些点获取圆的位姿
		for (int i = 0; i < circle_point3d.size(); i++)
		{
			for (int j = i; j < circle_point3d.size(); j++)
			{
				if (circle_point3d[i].y > circle_point3d[j].y)
				{
					cv::Point3d pt;
					pt = circle_point3d[i];
					circle_point3d[i] = circle_point3d[j];
					circle_point3d[j] = pt;
				}
			}
		}
		// 上下边际各五个点进行估计圆心深度
		double depth = 0;
		int Sample_num = 5;
		for (int i = 0; i < Sample_num; i++)
		{
			depth = depth + circle_point3d[i].z + circle_point3d[Sample_num - 1 - i].z;
		}
		depth = depth / 10.0;
		double x = (ellipse_raw.center.x - cam_intrinsics[2]) * depth / cam_intrinsics[0];
		double y = (ellipse_raw.center.y - cam_intrinsics[3]) * depth / cam_intrinsics[1];
		cv::Point3d position;
		position = cv::Point3d(x, y, depth);
		cout << "position:" << position.x << "\t"
			<< position.y << "\t"
			<< position.z << endl;
		return position;
	}
	//直线识别
	/*
		input: 灰度图像
		output:vector<LS> lines
		LS {
		cv::Point2d start;
		cv::Point2d end;
	};
	*/
	vector<LS> rec_l(cv::Mat& testImg)//input with grayimg可以是HSV处理的
	{
		ED testED = ED(testImg, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm
		EDLines testEDLines = EDLines(testED);
		vector<LS> lines = testEDLines.getLines();
		//显示图像
		//Mat lineImg = testEDLines.getLineImage();	//draws on an empty image
		//imshow("Line Image 1 - PRESS ANY KEY TO CONTINUE", lineImg);
		//waitKey();
		//lineImg = testEDLines.drawOnImage();	//draws on the input image
		//imshow("Line Image 2  - PRESS ANY KEY TO CONTINUE", lineImg);
		//waitKey();

		return lines;
	}
	vector<LS> rec_l_free(cv::Mat& testImg)
	{
		EDPF testEDPF = EDPF(testImg);
		EDLines testEDLines = EDLines(testEDPF);
		vector<LS> lines = testEDLines.getLines();
		return lines;
	}
		//识别杆子
	/*
		input: 彩色图像
		output:vector<mEllipse> ellipses
		mEllipse {
		cv::Point2d center;
		cv::Size axes;
		double theta;旋转角度顺时针为正，逆时针为负
	};
	*/
	//识别杆子
	/*
		input: 彩色图像
		output:vector<mEllipse> ellipses
		mEllipse {
		cv::Point2d center;
		cv::Size axes;
		double theta;旋转角度顺时针为正，逆时针为负
	};
	*/
	double dis_two(LS& line_raw, cv::Point2d& point_)
	{
		double distance;
		if (fabs(line_raw.start.x - line_raw.end.x) > 1)
		{
			double k = (line_raw.start.y - line_raw.end.y) / (line_raw.start.x - line_raw.end.x);
			double b = line_raw.start.y - k * line_raw.start.x;
			distance = fabs(point_.y - k * point_.x - b) / sqrt(1 + k * k);
		}
		else
		{
			distance = fabs(point_.x - line_raw.start.x);
		}
		return distance;
	}

	std::vector<LS> repair_Line(const std::vector<LS>& lines)
	{
		std::vector<LS> tmp_Lines = lines;
		std::vector<LS> repaired_Lines_raw;
		std::vector<LS> repaired_Lines;
		// 默认上方为起点
		for (std::vector<LS>::iterator i = tmp_Lines.begin(); i != tmp_Lines.end(); i++)
		{
			if (i->start.y > i->end.y)
			{
				cv::Point2d t = i->end;
				i->end = i->start;
				i->start = t;
			}
			// 赋值k
			if (fabs(i->start.x - i->end.x) > 1)
			{
				i->k = (i->start.y - i->end.y) / (i->start.x - i->end.x);
			}
			else
			{
				i->k = 1024;
			}
		}
		for (std::vector<LS>::iterator i = tmp_Lines.begin(); i != tmp_Lines.end(); i++)
		{
			if (fabs(i->k) > kPole)
			{
				for (std::vector<LS>::iterator j = tmp_Lines.begin(); j != tmp_Lines.end() && i->start.y != -1; j++)
				{
					if (j->start.y != -1 && j != i && fabs(fabs(i->k) - fabs(j->k)) < 1 / LS_k && dis_two(*i, j->start) < is_Line && dis_two(*i, j->end) < is_Line)
					{
						if (i->start.y > j->start.y)
						{
							i->start = j->start;
						}
						if (i->end.y < j->end.y)
						{
							i->end = j->end;
						}
						// 标志被检测过
						j->start.y = -1;
					}
				}
				cv::Point i_point = i->start - i->end;
				if (i->start.y != -1 && i_point.x * i_point.x + i_point.y * i_point.y > minPole)
				{
					// std::cout << "i:" << i - tmp_Lines.begin() << " k:" << i->k;
					// std::cout << " i_length:" << i_point.x * i_point.x + i_point.y * i_point.y;
					// std::cout << " iend_x:" << i->end.x << " iend_y:" << i->end.y;
					// std::cout << " istart_x:" << i->start.x << " istart_y:" << i->start.y << std::endl;
					repaired_Lines_raw.push_back(*i);
				}
			}
		}
		for (std::vector<LS>::iterator i = repaired_Lines_raw.begin(); i != repaired_Lines_raw.end(); i++)
		{
			for (std::vector<LS>::iterator j = repaired_Lines_raw.begin(); j != repaired_Lines_raw.end() && i->start.y != -1; j++)
			{
				if (j->start.y != -1 && j != i && fabs(1 / fabs(i->k) - 1 / fabs(j->k)) < LS_k && dis_two(*i, j->start) < is_Line && dis_two(*i, j->end) < is_Line)
				{
					if (i->start.y > j->start.y)
					{
						i->start = j->start;
					}
					if (i->end.y < j->end.y)
					{
						i->end = j->end;
					}
					// 标志被检测过
					j->start.y = -1;
				}
			}
			if (i->start.y != -1)
			{
				repaired_Lines.push_back(*i);
			}
		}
		return repaired_Lines;
	}

	std::vector<std::pair<LS, LS>> match_line(const std::vector<LS>& lines)
	{
		std::vector<std::pair<LS, LS>> pole;
		std::vector<LS> tmp_Lines = lines;
		std::vector<int> ismatch;
		ismatch.resize(lines.size(), 0);
		// std::cout << "***********" << std::endl;
		for (std::vector<LS>::iterator i = tmp_Lines.begin(); i != tmp_Lines.end(); i++)
		{
			if (ismatch[i - tmp_Lines.begin()] != 1)
			{
				cv::Point i_point = i->start - i->end;
				for (std::vector<LS>::iterator j = i + 1; j != tmp_Lines.end(); j++)
				{
					double ij_start = dis_two(*i, j->start);
					double ij_end = dis_two(*i, j->end);
					// std::cout << "i:" << i - tmp_Lines.begin() << " j:" << j - tmp_Lines.begin()
					//           << " ij_end:" << ij_end << " ij_start:" << ij_start << std::endl;
					if (ismatch[j - tmp_Lines.begin()] != 1 && fabs(ij_start - ij_end) < minDisDiff && fabs(ij_end) < maxDis && fabs(ij_end) > minDis)
					{
						cv::Point j_point = j->start - j->end;
						// std::cout << "i:" << i - tmp_Lines.begin() << " j:" << j - tmp_Lines.begin()
						//           << " pole:" << i_point.x * i_point.x + i_point.y * i_point.y << std::endl;
						double ratio_ = (double)(i_point.x * i_point.x + i_point.y * i_point.y) / (j_point.x * j_point.x + j_point.y * j_point.y);
						// std::cout << "i:" << i - tmp_Lines.begin() << " j:" << j - tmp_Lines.begin()
						//           << " pole:" << j_point.x * j_point.x + j_point.y * j_point.y;
						// std::cout << " ratio:" << ratio_ << std::endl;
						if (ratio_ < ::ratio && ratio_ > 1 / ::ratio)
						{
							pole.push_back(std::make_pair(*i, *j));
							ismatch[j - tmp_Lines.begin()] = 1;
							ismatch[i - tmp_Lines.begin()] = 1;
						}
					}
				}
			}
		}
		return pole;
	}

	cv::Point2f FindFoot(const cv::Point2f& pntSart, const cv::Point2f& pntEnd, const cv::Point2f& pA)
	{
		cv::Point2f pFoot;
		float k = 0.0;
		if (pntSart.x == pntEnd.x)
		{
			pFoot.x = pntSart.x;
			pFoot.y = pA.y;
			return pFoot;
		}
		k = (pntEnd.y - pntSart.y) * 1.0 / (pntEnd.x - pntSart.x);
		float A = k;
		float B = -1.0;
		float C = pntSart.y - k * pntSart.x;

		pFoot.x = (B * B * pA.x - A * B * pA.y - A * C) / (A * A + B * B);
		pFoot.y = (A * A * pA.y - A * B * pA.x - B * C) / (A * A + B * B);
		return pFoot;
	}
	// 根据得到的线段对得出杆子的矩形线段对
	std::vector<std::pair<LS, LS>> pole_rec(const std::vector<std::pair<LS, LS>>& lines)
	{
		std::vector<std::pair<LS, LS>> recs;
		// std::vector<std::pair<LS, LS>> recs_true; // 去除邻近的矩形
		for (int i = 0; i < lines.size(); i++)
		{
			std::pair<LS, LS> rec = lines[i];
			cv::Point2f start = FindFoot(lines[i].second.start, lines[i].second.end, lines[i].first.start);
			if (start.y < lines[i].second.start.y)
			{
				rec.second.start = start;
			}
			else
			{
				rec.first.start = FindFoot(lines[i].first.start, lines[i].first.end, lines[i].second.start);
			}
			cv::Point2f end = FindFoot(lines[i].second.start, lines[i].second.end, lines[i].first.end);
			if (end.y > lines[i].second.end.y)
			{
				rec.second.end = end;
			}
			else
			{
				rec.first.end = FindFoot(lines[i].first.start, lines[i].first.end, lines[i].second.end);
			}
			cv::Point2d width = rec.first.start - rec.second.start;
			cv::Point2d height = rec.first.start - rec.first.end;
			if (sqrt((height.x * height.x + height.y * height.y) / (width.x * width.x + width.y * width.y)) > aspect_ratio)
			{
				if (rec.first.start.x > rec.second.start.x)
				{
					LS t;
					t = rec.first;
					rec.first = rec.second;
					rec.second = t;
				}
				recs.push_back(rec);
			}
		}
		// 融合邻近的矩形
		// for (int i = 0; i < recs.size() - 1; i++)
		// {
		//     cv::Point2d i_center((recs[i].first.start.x + recs[i].second.end.x) / 2,
		//                          (recs[i].first.start.y + recs[i].second.end.y) / 2);
		//     for (int j = i; j < recs.size(); j++)
		//     {
		//         cv::Point2d j_center((recs[j].first.start.x + recs[j].second.end.x) / 2,
		//                              (recs[j].first.start.y + recs[j].second.end.y) / 2);
		//         if (true) // 保留接口，识别竖杆
		//         {
		//             if (fabs(i_center.x - j_center.x) < 20)
		//             {

		//             }
		//         }
		//     }
		// }

		return recs;
	}

	std::vector<std::pair<LS, LS>> detect_pole(cv::Mat& srcimg, cv::Mat& a, int type)
	{
		EDLines testED;
		if (type == CV_8UC1)
		{
			testED = EDLines(srcimg, _line_error, _min_line_len, _max_distance_between_two_lines, _max_error);
		}
		else if (type == CV_8UC3)
		{
			EDColor ab = EDColor(srcimg);
			testED = EDLines(ab, _line_error, _min_line_len, _max_distance_between_two_lines, _max_error);
		}
		std::vector<LS> lines = testED.getLines();
		std::vector<LS> rep = repair_Line(lines);
		std::vector<std::pair<LS, LS>> pole = match_line(rep);
		std::vector<std::pair<LS, LS>> pole_recs = pole_rec(pole);
		for (int i = 0; i < pole_recs.size(); i++)
		{
			line(srcimg, pole_recs[i].first.start, pole_recs[i].first.end, cv::Scalar(0), 1, cv::LINE_AA, 0);
			line(srcimg, pole_recs[i].second.start, pole_recs[i].second.end, cv::Scalar(0), 1, cv::LINE_AA, 0);
			line(srcimg, pole_recs[i].second.start, pole_recs[i].first.start, cv::Scalar(0), 1, cv::LINE_AA, 0);
			line(srcimg, pole_recs[i].second.end, pole_recs[i].first.end, cv::Scalar(0), 1, cv::LINE_AA, 0);
		}

		for (int i = 0; i < lines.size(); i++)
		{
			line(a, lines[i].start, lines[i].end, cv::Scalar(0), 1, cv::LINE_AA, 0);
		}
		return pole_recs;
	}
	std::vector<cv::Point3d> get_point(std::vector<std::pair<LS, LS>> pole_recs_, cv::Mat dep_img)
	{
		std::vector<cv::Point3d> depth_vec;
		std::vector<cv::Point3d> screened_pole;
		for (int i = 0; i < pole_recs_.size(); i++)
		{
			depth_vec.clear();
			int t = 0;
			double sum = 0;
			for (int v = std::max(pole_recs_[i].first.start.y, pole_recs_[i].second.start.y);
				v < std::min(pole_recs_[i].first.end.y, pole_recs_[i].second.end.y); v++)
			{
				for (int u = pole_recs_[i].first.start.x + (v - pole_recs_[i].first.start.y) / pole_recs_[i].first.k;
					u < pole_recs_[i].second.start.x + (v - pole_recs_[i].second.start.y) / pole_recs_[i].second.k;
					u++)
				{
					int depth_2048 =  dep_img.at<ushort>(v, u);
					//if (depth_2048 != 0)
					if (depth_2048 > 0 && depth_2048 < 3000)
					{
						cv::Point3d depth_xy;
						depth_xy.z = depth_2048;
						depth_xy.x = u;
						depth_xy.y = v;
						depth_vec.push_back(depth_xy);
					}
				}
			}
		}
		return depth_vec;
	}
	cv::Point3d ransac_double(std::vector<cv::Point3d> depth, double intri[4],
		double threshold, double ratio)
	{
		int num = depth.size() / ratio;
		std::vector<int> a;
		cv::Point3d average(-1, -1, -1);
		cout << "size:" << depth.size() << endl;
		for (int i = 0; i < depth.size(); i++)
		{
			a.clear();
			int num_i = 0;
			double sum = 0;
			for (int j = 0; j < depth.size(); j++)
			{
				if (fabs(depth[i].z - depth[j].z) < threshold)
				{
					num_i++;
					sum = sum + depth[j].z;
					a.push_back(1);
				}
				else
					a.push_back(0);
			}
			if (num_i > num)
			{
				average.z = sum / num_i;
				for (int i = 0; i < depth.size(); i++)
				{
					if (a[i] == 1)
					{
						average.x = (depth[i].x - intri[2]) * average.z / intri[0] + average.x;
						average.y = (depth[i].y - intri[3]) * average.z / intri[1] + average.y;
					}
				}
				average.x = (average.x + 1) / num_i;
				average.y = (average.y + 1) / num_i;
				cout << "x,y,z:" << average.x << "\t" << average.y << "\t" << average.z << endl;

				cout << "normal:" << num_i << endl;
				for (int i = 0; i < depth.size(); i++)
				{
					if (a[i] == 1)
					{
						//cout << depth[i].x << "\t" << depth[i].y << "\t" << depth[i].z << endl;
					}
				}
				cout << "abnormal:" << endl;
				for (int i = 0; i < depth.size(); i++)
				{
					if (a[i] != 1)
					{
						//cout << depth[i].x << "\t" << depth[i].y << "\t" << depth[i].z << endl;
					}
				}
				break;
			}
			//cout << num_i << endl;
		}
		
		return average;
	}

	std::vector<cv::Point3d> true_pole(std::vector<cv::Point3d> pole, cv::Point3d bottom_left,
		cv::Point3d top_right_front)
	{
		cout << "true_pole:" << endl;
		std::vector<cv::Point3d> true_pole_;
		int t = 1;
		for (int i = 0; i < pole.size(); i++)
		{
			if (pole[i].z > bottom_left.z && pole[i].z < top_right_front.z && pole[i].x > bottom_left.x && pole[i].x < top_right_front.x)
			{
				pole[i].y = t;
				// 判断是否有相同数据，有则取平均值
				for (int j = 0; j < true_pole_.size(); j++)
				{
					if (fabs(pole[i].x - true_pole_[j].x) < 0.1 && fabs(pole[i].z - true_pole_[j].z) < 0.1)
					{
						true_pole_[j].x = (pole[i].x + true_pole_[j].x) / 2;
						true_pole_[j].z = (pole[i].z + true_pole_[j].z) / 2;
						pole[i].z = -1;
						break;
					}
				}
				if (pole[i].z > 0)
				{
					true_pole_.push_back(pole[i]);
					cout << "x,y,z:" << pole[i].x << "\t" << pole[i].y << "\t" << pole[i].z << endl;
				}
				t++;
			}
		}
		// if (true_pole_.size() != pole_num)
		// {
		//     true_pole_.clear();
		// }
		for (int i = 0; i < true_pole_.size(); i++)
		{
			for (int j = i; j < true_pole_.size(); j++)
			{
				if (true_pole_[i].z > true_pole_[j].z)
				{
					cv::Point3d pt;
					pt = true_pole_[i];
					true_pole_[i] = true_pole_[j];
					true_pole_[j] = pt;
				}
			}
		}
		return true_pole_;
	}

	//彩色图像识别椭圆
	/*
		input: 彩色图像
		output:vector<mEllipse> ellipses
		mEllipse {
		cv::Point2d center;
		cv::Size axes;
		double theta;旋转角度顺时针为正，逆时针为负
	};
	*/
	mEllipse rec_ellipses_color(cv::Mat& testImg)
	{
		EDColor testEDColor = EDColor(testImg, 36, 4, 1.5, true); //last parameter for validation
		EDCircles colorCircle = EDCircles(testEDColor);
		vector<mEllipse> ellipses = colorCircle.getEllipses();
		//获取识别到的最大椭圆
		int noellipse = ellipses.capacity();
		int find_max_ellipses = 0;
		if (noellipse != 0)
		{
			for (int i = 1; i < noellipse; i++)
			{
				if (ellipses[find_max_ellipses].axes.area() < ellipses[i].axes.area())
					find_max_ellipses = i;
			}
			cout << "find ellipses, ellipses number == " + noellipse << endl;
		}
		if (noellipse == 0)
		{
			cout << "not find ellipses" << endl;
		}
		//显示图像;
		//Mat circleImg = colorCircle.drawResult(false, ImageStyle::ELLIPSES);//显示椭圆,使用colorimg时只能用false
		//imshow("Color Circle", circleImg);
		//waitKey();
		return ellipses[find_max_ellipses];
	}
	//彩色图像圆形识别
	/*
		input: 彩色图像
		output:vector<mCircle> circles
		mCircle {
		cv::Point2d center;
		double r;
	};
	*/
	mCircle rec_o_color(cv::Mat& testImg)
	{
		EDColor testEDColor = EDColor(testImg, 36, 4, 1.5, true); //last parameter for validation
		EDCircles colorCircle = EDCircles(testEDColor);
		vector<mCircle> circles = colorCircle.getCircles();
		//获取识别到的最大圆
		int nocircle = circles.capacity();
		int find_max_circle = 0;
		if (nocircle != 0)
		{
			for (int i = 1; i < nocircle; i++)
			{
				if (circles[find_max_circle].r < circles[i].r)
					find_max_circle = i;
			}
			cout << "find ellipses, ellipses number == " + nocircle << endl;
		}
		if (nocircle == 0)
		{
			cout << "not find circles" << endl;
		}
		return circles[find_max_circle];
	}

	//彩色图像直线识别
	/*
		input: 灰度图像
		output:vector<LS> lines
		LS {
		cv::Point2d start;
		cv::Point2d end;
	};
	*/
	vector<LS> rec_l_color(cv::Mat& testImg)//input with grayimg可以是HSV处理的
	{
		EDColor testEDColor = EDColor(testImg, 36, 4, 1.5, true); //last parameter for validation
		EDLines colorLine = EDLines(testEDColor);
		vector<LS> lines = colorLine.getLines();
		//显示图像,只能画空的，不能在原有图上画
		Mat lineImg = colorLine.getLineImage();	//draws on an empty image
		imshow("Line Image 2 - PRESS ANY KEY TO CONTINUE", lineImg);
		waitKey();
		return lines;
	}
	//获取边缘图像
	/*
	input: 灰度图像
	output:边缘图像
	*/
	Mat edgeImg(cv::Mat& testImg)//input with grayimg可以是HSV处理的
	{
		ED testED = ED(testImg, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm
		//Show resulting edge image
		Mat edgeImg = testED.getEdgeImage();
		return edgeImg;
	}
	//获取输入直线中竖直的直线
	/*
	input: vector<LS> line
	output:vector<LS> line
	*/
	vector<LS> getline_erect(vector<LS> line1)
	{
		vector<LS> line_erect;
		for (int i = 0; i < line1.size(); i++)
		{
			if (abs(line1[i].end.x - line1[i].start.x) < 10)
			{
				line_erect.push_back(line1[i]);
			}
		}
		//画图
		//这里需要传入画图的图像
		//Mat testImg = imread("D:/Desktop/code/zuanquan/img/new.jpg");
		//for (int i = 0; i < line_erect.size(); i++) {
		//	//if (lines[i].b < 0.1 && lines[i].invert == 1 && lines[i].b > -0.1) {
		//	line(testImg, line_erect[i].start, line_erect[i].end, Scalar(0,255,0), 1, LINE_AA, 0);
		//	//}
		//}
		//imshow("123",testImg);
		//waitKey();
		return line_erect;
	}

	//获取输入直线中水平的直线
	/*
	input: vector<LS> line
	output:vector<LS> line
	*/
	vector<LS> getline_level(vector<LS> line)
	{
		vector<LS> line_level;
		for (int i = 0; i < line.size(); i++)
		{
			if (abs(line[i].end.y - line[i].start.y) < 10)
			{
				line_level.push_back(line[i]);
			}
		}
		return line_level;
	}
	//识别直线opencv的方法，边缘提取
	/*
	input: 灰度图像
	output:直线
	*/
	std::vector<cv::Vec4i> rec_l_cv(cv::Mat& testImg)
	{
		std::vector<cv::Vec4i> line;
		// 进行边缘检测
		cv::Mat edges;
		cv::Canny(testImg, edges, 50, 150);
		cv::imshow("canny", edges);
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 300, 60);
		// 在原始图像上绘制直线段
		//for (size_t i = 0; i < lines.size(); i++)
		//{
		//	if (abs(lines[i][0] - lines[i][2]) < 20 && lines[i][0] > 50 && lines[i][0] < 440 && abs(lines[i][1] - lines[i][3]) > 400)
		//	{
		//		cv::Vec4i line = lines[i];
		//		cv::line(image_source, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
		//	}

		//}
		return lines;
	}
	//获取输入直线中竖直的直线_for_cv
	/*
	input: vector<cv::Vec4i> line
	output:vector<cv::Vec4i> line
	x1,y1,x2,y2
	*/
	std::vector<cv::Vec4i> getline_erect_cv(std::vector<cv::Vec4i> lines)
	{
		std::vector<cv::Vec4i> line_erect;
		//用于画图的
		//cv::Mat image_source = cv::imread("D:/Desktop/code/zuanquan/img/ganzi13.jpg");
		for (size_t i = 0; i < lines.size(); i++)
		{
			if (abs(lines[i][0] - lines[i][2]) < 20 && lines[i][0] > 50 && lines[i][0] < 440 && abs(lines[i][1] - lines[i][3]) > 400)
			{
				cv::Vec4i line = lines[i];
				line_erect.push_back(line);
				//cv::line(image_source, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
			}
		}
		//imshow("11", image_source);
		//waitKey();
		return line_erect;
	}
	//获取输入直线中水平的直线_for_cv
	/*
	input: vector<cv::Vec4i> line
	output:vector<cv::Vec4i> line
	x1,y1,x2,y2
	*/
	std::vector<cv::Vec4i> getline_level_cv(std::vector<cv::Vec4i> lines)
	{
		std::vector<cv::Vec4i> line_level;
		for (size_t i = 0; i < lines.size(); i++)
		{
			if (abs(lines[i][1] - lines[i][3]) < 20 && lines[i][1] > 50 && lines[i][1] < 440 && abs(lines[i][0] - lines[i][2]) > 400)
			{
				cv::Vec4i line = lines[i];
				line_level.push_back(line);
				//cv::line(image_source, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
			
			}
		}
		return line_level;
	}
	//识别矩形
	/*
	input: 灰度图像
	output:vector<Point>
	传出来的点每四个点是一个矩形
	*/
	Point rec_rectangles(cv::Mat& testImg,double& angle)
	{
		medianBlur(testImg, testImg, 3);
		Mat thresh;
		thresh = testImg;
		//threshold(testImg, thresh, 0, 255, THRESH_OTSU);//此处是加上一个阈值处理

		//imshow("123", thresh);
		//waitKey();
		vector<Point>squares;
		vector<Point>approx;
		vector<Point>approx_max;
		vector<vector<Point>>contours;
		Mat srcImg1;
		srcImg1 = thresh.clone();
		//此处可以是输入经过边缘检测的图
		// 不要用边缘检测的图，不然会去获得边缘那个很小的多边形
		//cv::Mat edges;
		//cv::Canny(thresh, edges, 50, 150);
		//imshow("123",edges);
		//waitKey();
		//srcImg1 = edges.clone();
		findContours(srcImg1, contours, RETR_TREE, CHAIN_APPROX_NONE);
		/*
		CV_RETR_EXTERNAL ：返回最外层轮廓， hierarchy[i][2]=hierarchy[i][3]=-1 
		CV_RETR_LIST ：返回所有的轮廓，但是没建立等级关系
		CV_RETR_CCOMP ：返回所有轮廓，包含两个层级结构
		CV_RETR_TREE ：返回所有轮廓，建立完整的层次结构

		CV_CHAIN_APPROX_NONE  ：存储轮廓的所有点
		CV_CHAIN_APPROX_SIMPLE ：不保存轮廓中水平、垂直、对角的线段，只保存轮廓的角点
		CV_CHAIN_APPROX_TC89_L1,CV_CHAIN_APPROX_TC89_KCOS ： 应用了 Teh-Chin 链近  似算法的一种存储风格，这个我也没搞懂
		*/
		Mat dstImg(srcImg1.rows, srcImg1.cols, CV_8UC3, Scalar(255, 255, 255));
		double Max_area = 0;
		int index_max = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			//approxPolyDP(contours[i], approx, arcLength(Mat(contours[i]), true) * 0.02, true);
			approxPolyDP(contours[i], approx, 16, true);
			/*
			* https://www.cnblogs.com/bjxqmy/p/12347265.html
			InputArray curve：输入曲线，数据类型可以为vector<Point>。
			OutputArray approxCurve：输出折线，数据类型可以为vector<Point>。
			double epsilon：判断点到相对应的line segment 的距离的阈值。（距离大于此阈值则舍弃，小于此阈值则保留，epsilon越小，折线的形状越“接近”曲线。）
			bool closed：曲线是否闭合的标志位。
			*/
			//cout << contourArea(Mat(approx)) << endl;
			if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)))
			{
				double minDist = 1e10;

				for (int i = 0; i < 4; i++)
				{
					Point side = approx[i] - approx[(i + 1) % 4];
					double squaredSideLength = side.dot(side);
					minDist = min(minDist, squaredSideLength);
				}
				if (minDist < 50)
					break;
				if (fabs(contourArea(Mat(approx))) > Max_area)
				{
					index_max = i;
					approx_max = approx;
					Max_area = fabs(contourArea(Mat(approx)));
				}
			}
		}
		for (int i = 0; i < 4; i++)
		{
			squares.push_back(Point(approx_max[i].x, approx_max[i].y));
		}
		Point rec_center;
		rec_center.x = (squares[0].x + squares[2].x) / 2;
		rec_center.y = (squares[0].y + squares[2].y) / 2;
		//算角度
		for (int j = 0; j < 4; j++)
		{
			for (int jj = j + 1; jj < 4; jj++)
			{
				if (squares[jj].y > squares[j].y)
				{
					Point pt = squares[j];
					squares[j] = squares[jj];
					squares[jj] = pt;
				}
			}
		}
		double a = abs(squares[3].y - squares[2].y);
		double b = abs(squares[3].x - squares[2].x);
		double cc = a / b;
		angle = atan(cc);
		////此处用于画图
		//for (size_t i = 0; i < squares.size(); i += 4)
		//{
		//	Point center;
		//	center.x = (squares[i].x + squares[i + 2].x) / 2;
		//	center.y = (squares[i].y + squares[i + 2].y) / 2;
		//	line(dstImg, squares[i], squares[i + 1], Scalar(0, 0, 255), 4);
		//	line(dstImg, squares[i + 1], squares[i + 2], Scalar(0, 0, 255), 4);
		//	line(dstImg, squares[i + 2], squares[i + 3], Scalar(0, 0, 255), 4);
		//	line(dstImg, squares[i + 3], squares[i], Scalar(0, 0, 255), 4);
		//	cout << "矩形中心" << (i + 1) % 4 << center << endl;
		//	circle(dstImg, center, 3, Scalar(0, 0, 255), -1);
		//}
		//imshow("123", dstImg);
		//waitKey();
		return rec_center;
	}
	//识别三角形
	/*
	input: 灰度图像
	output:vector<Point>
	传出来的点每三个点是一个三角形形
	*/
	vector<Point> rec_triangle(cv::Mat& testImg)
	{
		medianBlur(testImg, testImg, 3);
		Mat thresh;
		threshold(testImg, thresh, 0, 255, THRESH_OTSU);
		vector<Point>sanjiao;
		vector<Point>approx;
		vector<vector<Point>>contours;
		Mat srcImg1;
		srcImg1 = thresh.clone();
		findContours(srcImg1, contours, RETR_TREE, CHAIN_APPROX_NONE);
		Mat dstImg(srcImg1.rows, srcImg1.cols, CV_8UC3, Scalar(255, 255, 255));
		for (size_t i = 0; i < contours.size(); i++)
		{
			approxPolyDP(contours[i], approx, arcLength(Mat(contours[i]), true) * 0.1, true);
			if (approx.size() == 3 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)))
			{
				double minDist = 1e10;

				for (int i = 0; i < 3; i++)
				{
					Point side = approx[i] - approx[(i + 1) % 3];
					double squaredSideLength = side.dot(side);
					minDist = min(minDist, squaredSideLength);
				}
				if (minDist < 50)
					break;
				for (int i = 0; i < 3; i++)
					sanjiao.push_back(Point(approx[i].x, approx[i].y));
			}
		}
		//此处用于画图
		//for (size_t i = 0; i < sanjiao.size(); i += 3)
		//{
		//	Point center;
		//	center.x = (sanjiao[i].x + sanjiao[i + 1].x + sanjiao[i + 2].x) / 3;
		//	center.y = (sanjiao[i].y + sanjiao[i + 1].y + sanjiao[i + 2].y) / 3;
		//	line(dstImg, sanjiao[i], sanjiao[i + 1], Scalar(255, 0, 0), 4);
		//	line(dstImg, sanjiao[i + 1], sanjiao[i + 2], Scalar(255, 0, 0), 4);
		//	line(dstImg, sanjiao[i], sanjiao[i + 2], Scalar(255, 0, 0), 4);
		//	cout << "三角形中心" << (i + 1) % 3 << center << endl;
		//	circle(dstImg, center, 3, Scalar(255, 0, 0), -1);
		//}
		//imshow("123", dstImg);
		//waitKey(0);
		return sanjiao;
	}	
	//根据得到的点获取相对位置
	/*
	input: 像素点，像素点的深度，相机的内参矩阵
	output: cv::Point3d
		position.x,position.y,position.z
	*/
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

	//根据相对位置获取绝对位置
	/*
	input: 相对位置 position 飞机自身的绝对位置：drone_pos 
	output: 绝对位置 abs_position
		position.x,position.y,position.z
	*/
	// cv::Point3d get_abs_pose(cv::Point3d position, cv::Point3d drone_pos)
	// {
	// 	cv::Point3d abs_position;
	// 	abs_position.x = drone_pos.x + position.z;
	// 	abs_position.y = drone_pos.y - position.x;
	// 	abs_position.z = drone_pos.z - position.y;
	// 	return abs_position;
	// }

	//相机图像去畸变
	/*
	input: 畸变图像
	output: 去畸图像
	*/
	cv::Mat Dedistortion(cv::Mat testimg, double cam_intrinsics[4], double cam_dist[4],double width, double height)
	{
		Mat image_undistort;
		const cv::Mat K = (cv::Mat_<double>(3, 3) << cam_intrinsics[0], 0, cam_intrinsics[2],
			0, cam_intrinsics[1], cam_intrinsics[3],
			0, 0, 1);
		const cv::Mat D = (cv::Mat_<double>(4, 1) << cam_dist[0], cam_dist[1],
			cam_dist[2], cam_dist[3]);
		cv::Size imageSize(width, height);
		cv::Mat map1,map2;
		initUndistortRectifyMap(K, D, cv::Mat_<double>::eye(3, 3), K.clone(), imageSize, CV_16SC2, map1, map2);
		remap(testimg, image_undistort, map1, map2, cv::INTER_LINEAR);
		return image_undistort;
	}

}
