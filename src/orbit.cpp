#include<iostream>
#include "../include/orbit.h"
using namespace std;
void orbit::take_off(double height, double time)
{
    traj.clear();
    int point_num = time * sample_rate;
    for (int i = 0; i < point_num / 4; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = 0;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = 0;
        traj_point_.a_y = 0;
        traj_point_.a_z = 0.5;
        traj_point_.vel_x = 0;
        traj_point_.vel_y = 0;
        traj_point_.vel_z = height / time;
        traj_point_.pos_x = 0;
        traj_point_.pos_y = 0;
        traj_point_.pos_z = (i + 1) * height / (point_num + 1);
        traj.push_back(traj_point_);
    }
    for (int i = point_num / 4; i < point_num * 3 / 4; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = 0;
        traj_point_.a_y = 0;
        traj_point_.a_z = 0;
        traj_point_.vel_x = 0;
        traj_point_.vel_y = 0;
        traj_point_.vel_z = height / time;
        traj_point_.pos_x = 0;
        traj_point_.pos_y = 0;
        traj_point_.pos_z = (i + 1) * height / (point_num + 1);
        traj.push_back(traj_point_);
    }
    for (int i = point_num * 3 / 4; i < point_num; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = 0;
        traj_point_.a_y = 0;
        traj_point_.a_z = -4 * height / (time * time);
        traj_point_.vel_x = 0;
        traj_point_.vel_y = 0;
        traj_point_.vel_z = 4 * height / time - 4 * height * i / (point_num * time);
        traj_point_.pos_x = 0;
        traj_point_.pos_y = 0;
        traj_point_.pos_z = (i + 1) * height / (point_num + 1);
        traj.push_back(traj_point_);
    }
}
void orbit::hover(Eigen::Vector3d pos, double time, double yaw)
{
    traj.clear();
    int point_num = time * sample_rate;
    for (int i = 0; i < point_num; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = yaw;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = 0;
        traj_point_.a_y = 0;
        traj_point_.a_z = 0;
        traj_point_.vel_x = 0;
        traj_point_.vel_y = 0;
        traj_point_.vel_z = 0;
        traj_point_.pos_x = pos(0);
        traj_point_.pos_y = pos(1);
        traj_point_.pos_z = pos(2);
        traj.push_back(traj_point_);
    }
}
void orbit::generateYaw(Eigen::Vector3d pos, double time, double yaw, double cur_yaw)
{
    traj.clear();
    int point_num = time * sample_rate;
    for (int i = 0; i < point_num; i++)
    {
        traj_point traj_point_;
        // wrong
        traj_point_.yaw = cur_yaw + i * (yaw - cur_yaw) / point_num;
        traj_point_.yaw_rate = (yaw - cur_yaw) / time;
        traj_point_.a_x = 0;
        traj_point_.a_y = 0;
        traj_point_.a_z = 0;
        traj_point_.vel_x = 0;
        traj_point_.vel_y = 0;
        traj_point_.vel_z = 0;
        traj_point_.pos_x = pos(0);
        traj_point_.pos_y = pos(1);
        traj_point_.pos_z = pos(2);
        traj.push_back(traj_point_);
    }
}
void orbit::land(Eigen::Vector3d pos, double height_seg, double time, double yaw)
{
    traj.clear();
    double ratio = 0.0;
    if (pos(2) > 0)
    {
        ratio = height_seg / pos(2);
    }
    int point_num = time * sample_rate;
    Eigen::Vector3d pos_reg(pos(0), pos(1), height_seg);
    this->generateLine(pos, pos_reg, (1 - ratio) * time, yaw);
    int size_line = traj.size();
    for (int i = 0; i < ratio * point_num; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = yaw;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = 0;
        traj_point_.a_y = 0;
        traj_point_.a_z = 0;
        traj_point_.vel_x = 0;
        traj_point_.vel_y = 0;
        traj_point_.vel_z = -0.3;
        traj_point_.pos_x = pos(0);
        traj_point_.pos_y = pos(1);
        traj_point_.pos_z = -0.05;
        if (i + size_line > 0.95 * point_num)
        {
            traj_point_.vel_z = -0.3;
        }
        traj.push_back(traj_point_);
    }
}

void orbit::generateCircle(Eigen::Vector3d _pos, double radius, double yaw, double time, bool reverse, int n)
{
    traj.clear();
    int point_num = time * sample_rate;
    Eigen::Vector3d pos;
    pos(0) = _pos(0) + radius * cos(yaw);
    pos(1) = _pos(1) + radius * sin(yaw);
    pos(2) = _pos(2);
    if (reverse)
    {
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < point_num; i++)
            {
                traj_point traj_point_;
                double theta = i * 2 * PI / point_num + yaw;
                traj_point_.pos_x = pos(0) - radius * cos(theta);
                traj_point_.pos_y = pos(1) - radius * sin(theta);
                traj_point_.pos_z = pos(2);
                if (theta <= PI)
                    traj_point_.yaw = theta;
                else if (theta > PI)
                    traj_point_.yaw = theta - 2 * PI;
                else
                {
                }
                traj_point_.vel_x = 0;
                traj_point_.vel_y = -radius * 2 * PI / time;
                traj_point_.vel_z = 0;
                traj_point_.yaw_rate = 2 * PI / time;
                traj_point_.a_x = abs(traj_point_.vel_y * traj_point_.vel_y) / radius;
                traj_point_.a_y = 0;
                traj_point_.a_z = 0;
                if (j == 0 && i < (int)point_num * 0.2)
                {
                    traj_point_.vel_y = traj_point_.vel_y * 5 * i / point_num;
                    traj_point_.yaw_rate = traj_point_.yaw_rate * 5 * i / point_num;
                }
                if (j == n - 1 && i > (int)point_num * 0.8)
                {
                    traj_point_.vel_y = traj_point_.vel_y * 5 * (point_num - i) / point_num;
                    traj_point_.yaw_rate = traj_point_.yaw_rate * 5 * (point_num - i) / point_num;
                }
                traj.push_back(traj_point_);
            }
        }
    }
    else
    {
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < point_num; i++)
            {
                traj_point traj_point_;
                double theta = -i * 2 * PI / point_num + yaw;
                traj_point_.pos_x = pos(0) - radius * cos(theta);
                traj_point_.pos_y = pos(1) - radius * sin(theta);
                traj_point_.pos_z = pos(2);
                if (theta <= PI)
                    traj_point_.yaw = theta;
                else if (theta > PI)
                    traj_point_.yaw = theta + 2 * PI;
                else
                {
                }
                traj_point_.vel_x = 0;
                traj_point_.vel_y = radius * 2 * PI / time;
                traj_point_.vel_z = 0;
                traj_point_.yaw_rate = -2 * PI / time;
                traj_point_.a_x = abs(traj_point_.vel_y * traj_point_.vel_y) / radius;
                traj_point_.a_y = 0;
                traj_point_.a_z = 0;
                if (j == 0 && i < (int)point_num * 0.2)
                {
                    traj_point_.vel_y = traj_point_.vel_y * 5 * i / point_num;
                    traj_point_.yaw_rate = traj_point_.yaw_rate * 5 * i / point_num;
                }
                if (j == n - 1 && i > (int)point_num * 0.8)
                {
                    traj_point_.vel_y = traj_point_.vel_y * 5 * (point_num - i) / point_num;
                    traj_point_.yaw_rate = traj_point_.yaw_rate * 5 * (point_num - i) / point_num;
                }
                traj.push_back(traj_point_);
            }
        }
    }
}

void orbit::generateLine(Eigen::Vector3d pos, Eigen::Vector3d vec, double length, double time, double yaw)
{
    traj.clear();
    int point_num = time * sample_rate;
    Eigen::Vector3d vec_norm = vec / vec.norm();
    Eigen::Vector3d vec_real;
    vec_real << cos(yaw) * vec_norm(0) - sin(yaw) * vec_norm(1),
        sin(yaw) * vec_norm(0) + cos(yaw) * vec_norm(1),
        vec_norm(2);
    double vel_max = 2 * length / time;
    for (int i = 0; i < point_num / 2; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = yaw;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = 2 * vel_max * vec_norm(0) / time;
        traj_point_.a_y = 2 * vel_max * vec_norm(1) / time;
        traj_point_.a_z = 2 * vel_max * vec_norm(2) / time;
        traj_point_.vel_x = 2 * vel_max * vec_norm(0) * i / point_num;
        traj_point_.vel_y = 2 * vel_max * vec_norm(1) * i / point_num;
        traj_point_.vel_z = 2 * vel_max * vec_norm(2) * i / point_num;
        traj_point_.pos_x = pos(0) + 2 * vec_real(0) * length * pow((double)i / point_num, 2);
        traj_point_.pos_y = pos(1) + 2 * vec_real(1) * length * pow((double)i / point_num, 2);
        traj_point_.pos_z = pos(2) + 2 * vec_real(2) * length * pow((double)i / point_num, 2);
        traj.push_back(traj_point_);
    }

    for (int i = point_num / 2; i < point_num; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = yaw;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = -2 * vel_max * vec_norm(0) / time;
        traj_point_.a_y = -2 * vel_max * vec_norm(1) / time;
        traj_point_.a_z = -2 * vel_max * vec_norm(2) / time;
        traj_point_.vel_x = 2 * vel_max * vec_norm(0) - 2 * vel_max * vec_norm(0) * (i + 1) / point_num;
        traj_point_.vel_y = 2 * vel_max * vec_norm(1) - 2 * vel_max * vec_norm(1) * (i + 1) / point_num;
        traj_point_.vel_z = 2 * vel_max * vec_norm(2) - 2 * vel_max * vec_norm(2) * (i + 1) / point_num;
        double shift = -2 * length * pow((double)(i + 1 - point_num / 2) / point_num, 2) +
                       2 * length * (i + 1 - point_num / 2) / point_num;
        traj_point_.pos_x = pos(0) + shift * vec_real(0) + length * vec_real(0) / 2;
        traj_point_.pos_y = pos(1) + shift * vec_real(1) + length * vec_real(1) / 2;
        traj_point_.pos_z = pos(2) + shift * vec_real(2) + length * vec_real(2) / 2;
        traj.push_back(traj_point_);
    }
}

void orbit::generateLine(Eigen::Vector3d curpos, Eigen::Vector3d refpos, double time, double yaw)
{
    traj.clear();
    int point_num = time * sample_rate;
    Eigen::Vector3d vec_real = (refpos - curpos) / (refpos - curpos).norm();
    Eigen::Vector3d vec_norm;
    vec_norm << cos(yaw) * vec_real(0) + sin(yaw) * vec_real(1),
        cos(yaw) * vec_real(1) - sin(yaw) * vec_real(0),
        vec_real(2);
    double length = (curpos - refpos).norm();
    double vel_max = 2 * length / time;
    std::cout << "vec_real:" << vec_real.transpose() << std::endl;
    std::cout << "length:" << length << std::endl;
    std::cout << "vel_max:" << vel_max << std::endl;
    for (int i = 0; i < point_num / 2; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = yaw;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = 2 * vel_max * vec_norm(0) / time;
        traj_point_.a_y = 2 * vel_max * vec_norm(1) / time;
        traj_point_.a_z = 2 * vel_max * vec_norm(2) / time;
        traj_point_.vel_x = 2 * vel_max * vec_norm(0) * i / point_num;
        traj_point_.vel_y = 2 * vel_max * vec_norm(1) * i / point_num;
        traj_point_.vel_z = 2 * vel_max * vec_norm(2) * i / point_num;
        traj_point_.pos_x = curpos(0) + 2 * vec_real(0) * length * pow((double)i / point_num, 2);
        traj_point_.pos_y = curpos(1) + 2 * vec_real(1) * length * pow((double)i / point_num, 2);
        traj_point_.pos_z = curpos(2) + 2 * vec_real(2) * length * pow((double)i / point_num, 2);
        traj.push_back(traj_point_);
    }
    for (int i = point_num / 2; i < point_num; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = yaw;
        traj_point_.yaw_rate = 0;
        traj_point_.a_x = -2 * vel_max * vec_norm(0) / time;
        traj_point_.a_y = -2 * vel_max * vec_norm(1) / time;
        traj_point_.a_z = -2 * vel_max * vec_norm(2) / time;
        traj_point_.vel_x = 2 * vel_max * vec_norm(0) - 2 * vel_max * vec_norm(0) * (i + 1) / point_num;
        traj_point_.vel_y = 2 * vel_max * vec_norm(1) - 2 * vel_max * vec_norm(1) * (i + 1) / point_num;
        traj_point_.vel_z = 2 * vel_max * vec_norm(2) - 2 * vel_max * vec_norm(2) * (i + 1) / point_num;
        double shift = -2 * length * pow((double)(i + 1 - point_num / 2) / point_num, 2) +
                       2 * length * (i + 1 - point_num / 2) / point_num;
        traj_point_.pos_x = curpos(0) + shift * vec_real(0) + length * vec_real(0) / 2;
        traj_point_.pos_y = curpos(1) + shift * vec_real(1) + length * vec_real(1) / 2;
        traj_point_.pos_z = curpos(2) + shift * vec_real(2) + length * vec_real(2) / 2;
        traj.push_back(traj_point_);
    }
}

Eigen::Vector3d orbit::centerCircle3d(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3, double &radius)
{
    Eigen::Vector3d center;
    double a1 = p1[1] * p2[2] - p2[1] * p1[2] - p1[1] * p3[2] +
                p3[1] * p1[2] + p2[1] * p3[2] - p3[1] * p2[2];
    double b1 = -(p1[0] * p2[2] - p2[0] * p1[2] - p1[0] * p3[2] +
                  p3[0] * p1[2] + p2[0] * p3[2] - p3[0] * p2[2]);
    double c1 = (p1[0] * p2[1] - p2[0] * p1[1] - p1[0] * p3[1] +
                 p3[0] * p1[1] + p2[0] * p3[1] - p3[0] * p2[1]);
    double d1 = -(p1[0] * p2[1] * p3[2] - p1[0] * p3[1] * p2[2] -
                  p2[0] * p1[1] * p3[2] + p2[0] * p3[1] * p1[2] +
                  p3[0] * p1[1] * p2[2] - p3[0] * p2[1] * p1[2]);

    double a2 = 2 * (p2[0] - p1[0]);
    double b2 = 2 * (p2[1] - p1[1]);
    double c2 = 2 * (p2[2] - p1[2]);
    double d2 = p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2] -
                p2[0] * p2[0] - p2[1] * p2[1] - p2[2] * p2[2];

    double a3 = 2 * (p3[0] - p1[0]);
    double b3 = 2 * (p3[1] - p1[1]);
    double c3 = 2 * (p3[2] - p1[2]);
    double d3 = p1[0] * p1[0] + p1[1] * p1[1] + p1[2] * p1[2] -
                p3[0] * p3[0] - p3[1] * p3[1] - p3[2] * p3[2];

    center[0] = -(b1 * c2 * d3 - b1 * c3 * d2 -
                  b2 * c1 * d3 + b2 * c3 * d1 +
                  b3 * c1 * d2 - b3 * c2 * d1) /
                (a1 * b2 * c3 - a1 * b3 * c2 -
                 a2 * b1 * c3 + a2 * b3 * c1 +
                 a3 * b1 * c2 - a3 * b2 * c1);
    center[1] = (a1 * c2 * d3 - a1 * c3 * d2 -
                 a2 * c1 * d3 + a2 * c3 * d1 +
                 a3 * c1 * d2 - a3 * c2 * d1) /
                (a1 * b2 * c3 - a1 * b3 * c2 -
                 a2 * b1 * c3 + a2 * b3 * c1 +
                 a3 * b1 * c2 - a3 * b2 * c1);
    center[2] = -(a1 * b2 * d3 - a1 * b3 * d2 -
                  a2 * b1 * d3 + a2 * b3 * d1 +
                  a3 * b1 * d2 - a3 * b2 * d1) /
                (a1 * b2 * c3 - a1 * b3 * c2 -
                 a2 * b1 * c3 + a2 * b3 * c1 +
                 a3 * b1 * c2 - a3 * b2 * c1);
    radius = sqrt((p1[0] - center[0]) * (p1[0] - center[0]) +
                  (p1[1] - center[1]) * (p1[1] - center[1]) +
                  (p1[2] - center[2]) * (p1[2] - center[2]));
    return center;
}

// B-Spline基函数
double orbit::basis(int i, int k, double t, const std::vector<double> &knots)
{
    if (k == 0)
    {
        if (t >= knots[i] && t < knots[i + 1])
            return 1.0;
        return 0.0;
    }

    double denom1 = knots[i + k] - knots[i];
    double denom2 = knots[i + k + 1] - knots[i + 1];
    double coeff1 = 0.0, coeff2 = 0.0;

    if (denom1 > 0)
        coeff1 = ((t - knots[i]) / denom1) * basis(i, k - 1, t, knots);

    if (denom2 > 0)
        coeff2 = ((knots[i + k + 1] - t) / denom2) * basis(i + 1, k - 1, t, knots);

    return coeff1 + coeff2;
}

// B-Spline曲线拟合
std::vector<Eigen::Vector3d> orbit::bsplineFitting(const std::vector<Eigen::Vector3d> &controlPoints, int degree,
                                                   int numSamples)
{
    int numControlPoints = controlPoints.size();
    int numKnots = numControlPoints + degree + 1;
    std::vector<Eigen::Vector3d> bspline_points;
    std::vector<double> knots(numKnots);
    for (int i = 0; i < numKnots; ++i)
    {
        if (i < degree)
            knots[i] = 0.0;
        else if (i >= numKnots - degree)
            knots[i] = 1.0;
        else
            knots[i] = (i - degree + 1.0) / (numControlPoints - degree + 1.0);
    }

    double tStep = 1.0 / (numSamples - 1);

    for (int i = 0; i < numSamples - 1; ++i)
    {
        double t = i * tStep;
        double x = 0.0, y = 0.0, z = 0.0;

        for (int j = 0; j < numControlPoints; ++j)
        {
            double coeff = basis(j, degree, t, knots);
            x += coeff * controlPoints[j][0];
            y += coeff * controlPoints[j][1];
            z += coeff * controlPoints[j][2];
        }
        bspline_points.push_back(Eigen::Vector3d(x, y, z));
    }

    return bspline_points;
}

void orbit::generateBspline(const std::vector<Eigen::Vector3d> &controlPoints,
                            int degree, double time, double yaw, int num)
{
    traj.clear();
    int numSamples = time * sample_rate + 1;
    int numPoints = controlPoints.size();
    std::vector<Eigen::Vector3d> originalPoints = controlPoints;
    Eigen::Vector3d first_point = originalPoints[0];
    for (int i = 0; i < originalPoints.size(); i++)
    {
        originalPoints[i] = originalPoints[i] - first_point;
        std::cout << "i:" << i << "\tpoint:" << originalPoints[i][0] << "  "
                  << originalPoints[i][1] << "  "
                  << originalPoints[i][2] << "  " << std::endl;
    }

    // 控制点扩充
    while (numPoints < num)
    {
        std::vector<Eigen::Vector3d> originalPoints_;
        for (int i = 0; i < originalPoints.size() - 1; i++)
        {
            Eigen::Vector3d xy_;
            xy_[0] = (originalPoints[i][0] + originalPoints[i + 1][0]) / 2;
            xy_[1] = (originalPoints[i][1] + originalPoints[i + 1][1]) / 2;
            xy_[2] = (originalPoints[i][2] + originalPoints[i + 1][2]) / 2;
            originalPoints_.push_back(originalPoints[i]);
            originalPoints_.push_back(xy_);
        }
        originalPoints_.push_back(*(originalPoints.end() - 1));
        originalPoints = originalPoints_;
        numPoints = numPoints * 2 - 1;
    }

    std::vector<Eigen::Vector3d> extendedControlPoints = originalPoints;
    extendedControlPoints.insert(extendedControlPoints.begin(), originalPoints.front());
    extendedControlPoints.push_back(originalPoints.back());

    std::vector<Eigen::Vector3d> bspline_points = bsplineFitting(extendedControlPoints, degree, numSamples);
    bspline_points.push_back(originalPoints.back());
    bspline_points.push_back(originalPoints.back());

    for (int i = 0; i < bspline_points.size(); i++)
    {
        bspline_points[i] = bspline_points[i] + first_point;
    }

    double length = 0.0;
    for (int i = 0; i < bspline_points.size() - 1; i++)
    {
        length = length + (bspline_points[i] - bspline_points[i + 1]).norm();
    }
    std::cout << "length:" << length << std::endl;

    double aver_vel = length / time;
    std::cout << "average vel:" << aver_vel << std::endl;

    double last_yaw = yaw; // 上一个点的转角
    double vel = aver_vel, stop_vel = 0;
    for (int i = 0; i < bspline_points.size() - 2; ++i)
    {
        Eigen::Vector3d v1 = bspline_points[i];
        Eigen::Vector3d v2 = bspline_points[i + 1];
        Eigen::Vector3d v3 = bspline_points[i + 2];
        // 计算两个边向量并叉乘得到法向量
        Eigen::Vector3d v21 = v2 - v1;
        Eigen::Vector3d v31 = v3 - v1;
        Eigen::Vector3d normalVec = v21.cross(v31);
        // 开始计算点速度与加速度
        traj_point traj_point_;
        if (i > (bspline_points.size() - 2) - Stop * sample_rate) // 开始减速
        {
            vel = (bspline_points.size() - 3 - i) * stop_vel / (Stop * sample_rate);
        }
        else if (i < (bspline_points.size() - 2) - Stop * sample_rate)
        {
            vel = v21.norm() * sample_rate;
        }
        else
        {
            vel = v21.norm() * sample_rate;
            stop_vel = vel;
        }
        // 代表正在转向
        if (abs(normalVec.norm()) > 1e-6 || normalVec.norm() / (v21.norm() * v31.norm()) > Curvature_sur)
        {
            double r;
            Eigen::Vector3d center = centerCircle3d(v1, v2, v3, r);
            Eigen::Vector3d radial_rel = center - v1;                     // 向心方向
            Eigen::Vector3d tangential_rel = radial_rel.cross(normalVec); // 切向方向
            // Eigen::Vector3d radial(cos(yaw) * radial_rel(0) + sin(yaw) * radial_rel(1),
            //                        cos(yaw) * radial_rel(1) - sin(yaw) * radial_rel(0),
            //                        radial_rel(2));
            // Eigen::Vector3d tangential(cos(yaw) * tangential_rel(0) + sin(yaw) * tangential_rel(1),
            //                            cos(yaw) * tangential_rel(1) - sin(yaw) * tangential_rel(0),
            //                            tangential_rel(2));
            // tangential.normalize();
            // radial.normalize();
            traj_point_.yaw = atan2(tangential_rel(1), tangential_rel(0));
            traj_point_.yaw_rate = (traj_point_.yaw - last_yaw) * sample_rate;
            last_yaw = traj_point_.yaw;

            Eigen::Vector3d last_oren(cos(last_yaw), sin(last_yaw), 0);
            Eigen::Matrix3d R =
                Eigen::Quaterniond::FromTwoVectors(last_oren, tangential_rel).toRotationMatrix();

            Eigen::Vector3d Vec_tang = R * Eigen::Vector3d(1, 0, 0);
            Eigen::Vector3d Vec_ran = R * Eigen::Vector3d(0, 1, 0);

            Vec_tang.normalize();
            Vec_ran.normalize();

            traj_point_.vel_x = Vec_tang(0) * vel;
            traj_point_.vel_y = Vec_tang(1) * vel;
            traj_point_.vel_z = Vec_tang(2) * vel;

            traj_point_.a_x = Vec_ran(0) * vel * vel / r;
            traj_point_.a_y = Vec_ran(1) * vel * vel / r;
            traj_point_.a_z = Vec_ran(2) * vel * vel / r;

            traj_point_.pos_x = bspline_points[i](0);
            traj_point_.pos_y = bspline_points[i](1);
            traj_point_.pos_z = bspline_points[i](2);
        }
        else
        {
            // Eigen::Vector3d vec_rel = v2 - v1;
            // Eigen::Vector3d vec(cos(yaw) * vec_rel(0) + sin(yaw) * vec_rel(1),
            //                     cos(yaw) * vec_rel(1) - sin(yaw) * vec_rel(0),
            //                     vec_rel(2));
            // vec.normalize();
            traj_point_.yaw = atan2(v21(1), v21(0));
            traj_point_.yaw_rate = (traj_point_.yaw - last_yaw) * sample_rate;
            last_yaw = traj_point_.yaw;

            double roll = atan2(v21(2), sqrt(v21(0) * v21(0) + v21(1) * v21(1)));
            traj_point_.vel_x = vel * cos(roll);
            traj_point_.vel_y = 0;
            traj_point_.vel_z = vel * sin(roll);

            traj_point_.a_x = 0;
            traj_point_.a_y = 0;
            traj_point_.a_z = 0;

            traj_point_.pos_x = bspline_points[i](0);
            traj_point_.pos_y = bspline_points[i](1);
            traj_point_.pos_z = bspline_points[i](2);
        }
        traj.push_back(traj_point_);
    }
    // 终点
    for (int i = 0; i < Stop * sample_rate; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = last_yaw;
        traj_point_.yaw_rate = 0;
        traj_point_.vel_x = 0;
        traj_point_.vel_y = 0;
        traj_point_.vel_z = 0;
        traj_point_.a_x = 0;
        traj_point_.a_y = 0;
        traj_point_.a_z = 0;
        traj_point_.pos_x = bspline_points[bspline_points.size() - 2](0);
        traj_point_.pos_y = bspline_points[bspline_points.size() - 2](1);
        traj_point_.pos_z = bspline_points[bspline_points.size() - 2](2);
        traj.push_back(traj_point_);
    }
}

void orbit::generateBspline(Eigen::Vector3d curpos, const std::vector<Eigen::Vector3d> &controlPoints,
                            int degree, double time, double yaw, int num)
{
    traj.clear();
    int numSamples = time * sample_rate + 1;
    std::vector<Eigen::Vector3d> originalPoints = controlPoints;
    int numPoints = controlPoints.size();
    // 控制点扩充
    while (numPoints < num)
    {
        std::vector<Eigen::Vector3d> originalPoints_;
        for (int i = 0; i < originalPoints.size() - 1; i++)
        {
            Eigen::Vector3d xy_;
            xy_[0] = (originalPoints[i][0] + originalPoints[i + 1][0]) / 2;
            xy_[1] = (originalPoints[i][1] + originalPoints[i + 1][1]) / 2;
            xy_[2] = (originalPoints[i][2] + originalPoints[i + 1][2]) / 2;
            originalPoints_.push_back(originalPoints[i]);
            originalPoints_.push_back(xy_);
        }
        originalPoints_.push_back(*(originalPoints.end() - 1));
        originalPoints = originalPoints_;
        numPoints = numPoints * 2 - 1;
    }
    std::vector<Eigen::Vector3d> extendedControlPoints = originalPoints;
    extendedControlPoints.insert(extendedControlPoints.begin(), originalPoints.front());
    extendedControlPoints.push_back(originalPoints.back());

    std::vector<Eigen::Vector3d> bspline_points = bsplineFitting(extendedControlPoints, degree, numSamples);
    bspline_points.push_back(originalPoints.back());
    bspline_points.push_back(originalPoints.back());

    double length = 0.0;
    for (int i = 0; i < bspline_points.size() - 1; i++)
    {
        length = length + (bspline_points[i] - bspline_points[i + 1]).norm();
    }
    std::cout << "length:" << length << std::endl;

    double aver_vel = length / time;
    std::cout << "average vel:" << aver_vel << std::endl;

    double vel = aver_vel, stop_vel = 0;
    double last_yaw = yaw;
    for (int i = 0; i < bspline_points.size() - 2; ++i)
    {
        Eigen::Vector3d v1 = bspline_points[i];
        Eigen::Vector3d v2 = bspline_points[i + 1];
        Eigen::Vector3d v3 = bspline_points[i + 2];
        // 计算两个边向量并叉乘得到法向量
        Eigen::Vector3d v21 = v2 - v1;
        Eigen::Vector3d v31 = v3 - v1;
        Eigen::Vector3d normalVec = v21.cross(v31);
        // 开始计算点速度与加速度
        traj_point traj_point_;
        if (i > (bspline_points.size() - 2) - Stop * sample_rate) // 开始减速
        {
            vel = (bspline_points.size() - 3 - i) * stop_vel / (Stop * sample_rate);
        }
        else if (i < (bspline_points.size() - 2) - Stop * sample_rate)
        {
            vel = v21.norm() * sample_rate;
        }
        else
        {
            vel = v21.norm() * sample_rate;
            stop_vel = vel;
        }
        // 代表正在转向
        if (abs(normalVec.norm()) > 1e-6 || normalVec.norm() / (v21.norm() * v31.norm()) > Curvature_sur)
        {
            double r;
            Eigen::Vector3d center = centerCircle3d(v1, v2, v3, r);
            Eigen::Vector3d radial = (center - v1) / (center - v1).norm(); // 向心方向
            Eigen::Vector3d tangential = radial.cross(normalVec);
            Eigen::Vector3d radial_rel(cos(yaw) * radial(0) + sin(yaw) * radial(1),
                                       cos(yaw) * radial(1) - sin(yaw) * radial(0),
                                       radial(2));
            Eigen::Vector3d tangential_rel(cos(yaw) * tangential(0) + sin(yaw) * tangential(1),
                                           cos(yaw) * tangential(1) - sin(yaw) * tangential(0),
                                           tangential(2));
            tangential_rel.normalize();
            radial_rel.normalize();
            traj_point_.yaw = atan2(tangential_rel(1), tangential_rel(0));
            traj_point_.yaw_rate = (traj_point_.yaw - last_yaw) * sample_rate;
            last_yaw = traj_point_.yaw;

            Eigen::Vector3d last_oren(cos(last_yaw), sin(last_yaw), 0);
            Eigen::Matrix3d R =
                Eigen::Quaterniond::FromTwoVectors(last_oren, tangential_rel).toRotationMatrix();

            Eigen::Vector3d Vec_tang = R * Eigen::Vector3d(1, 0, 0);
            Eigen::Vector3d Vec_ran = R * Eigen::Vector3d(0, 1, 0);

            Vec_tang.normalize();
            Vec_ran.normalize();

            traj_point_.vel_x = vel * Vec_tang(0);
            traj_point_.vel_y = vel * Vec_tang(1);
            traj_point_.vel_z = vel * Vec_tang(2);
            traj_point_.a_x = Vec_ran(0) * vel * vel / r;
            traj_point_.a_y = Vec_ran(1) * vel * vel / r;
            traj_point_.a_z = Vec_ran(2) * vel * vel / r;
            traj_point_.pos_x = bspline_points[i](0) + curpos(0);
            traj_point_.pos_y = bspline_points[i](1) + curpos(1);
            traj_point_.pos_z = bspline_points[i](2) + curpos(2);
        }
        else
        {
            traj_point_.yaw = yaw + atan2(v21(1), v21(0));
            traj_point_.yaw_rate = (traj_point_.yaw - last_yaw) * sample_rate;
            last_yaw = traj_point_.yaw;
            traj_point_.yaw_rate = 0;

            double roll = atan2(v21(2), sqrt(v21(0) * v21(0) + v21(1) * v21(1)));
            traj_point_.vel_x = vel * cos(roll);
            traj_point_.vel_y = 0;
            traj_point_.vel_z = vel * sin(roll);

            traj_point_.a_x = 0;
            traj_point_.a_y = 0;
            traj_point_.a_z = 0;
            traj_point_.pos_x = bspline_points[i](0) + curpos(0);
            traj_point_.pos_y = bspline_points[i](1) + curpos(1);
            traj_point_.pos_z = bspline_points[i](2) + curpos(2);
        }
        traj.push_back(traj_point_);
    }
    // 终点
    for (int i = 0; i < Stop * sample_rate; i++)
    {
        traj_point traj_point_;
        traj_point_.yaw = last_yaw;
        traj_point_.yaw_rate = 0;
        traj_point_.vel_x = 0;
        traj_point_.vel_y = 0;
        traj_point_.vel_z = 0;
        traj_point_.a_x = 0;
        traj_point_.a_y = 0;
        traj_point_.a_z = 0;
        traj_point_.pos_x = bspline_points[bspline_points.size() - 2](0);
        traj_point_.pos_y = bspline_points[bspline_points.size() - 2](1);
        traj_point_.pos_z = bspline_points[bspline_points.size() - 2](2);
        traj.push_back(traj_point_);
    }
}