#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>

#define Stop 0.3
#define PI 3.1415926535
struct traj_point
{
    double pos_x, pos_y, pos_z;
    double vel_x, vel_y, vel_z;
    double a_x, a_y, a_z;
    double yaw;
    double yaw_rate;
};

class orbit
{
public:
    std::vector<traj_point> traj;
    double Curvature_sur;
    int sample_rate;
    orbit()
    {
        sample_rate = 10;
        Curvature_sur = 0.001;
    };
    orbit(int sample_rate_, double _Curvature_sur = 0.001)
    {
        sample_rate = sample_rate_;
        Curvature_sur = _Curvature_sur;
    };
    // 起飞
    void take_off(double height, double time);
    // 下降
    void land(Eigen::Vector3d pos, double height_seg, double time, double yaw);
    // 悬停
    void hover(Eigen::Vector3d pos, double time, double yaw);
    // 转角
    void generateYaw(Eigen::Vector3d pos, double time, double yaw, double cur_yaw);
    // if reverse 逆时针 if !reverse 顺时针
    // 圆圈 默认飞机正对杆子
    void generateCircle(Eigen::Vector3d pos, double radius, double yaw, double time, bool reverse, int n);
    // 直线 给定方向,默认飞机朝向直线方向
    void generateLine(Eigen::Vector3d pos, Eigen::Vector3d vec, double length, double time, double yaw);
    // 直线 给定目标点
    void generateLine(Eigen::Vector3d curpos, Eigen::Vector3d refpos, double time, double yaw);
    // B样条曲线
    Eigen::Vector3d centerCircle3d(Eigen::Vector3d p1, Eigen::Vector3d p2,
                                   Eigen::Vector3d p3, double &radius);     // 求圆心的位置及半径
    double basis(int i, int k, double t, const std::vector<double> &knots); // B-Spline基函数
    std::vector<Eigen::Vector3d> bsplineFitting(const std::vector<Eigen::Vector3d> &controlPoints,
                                                int degree, int numSamples); // B-Spline曲线拟合
    // 产生绝对位姿下的BSpline控制点
    void generateBspline(const std::vector<Eigen::Vector3d> &controlPoints,
                         int degree, double time, double yaw,int num);
    // 产生相对位姿下的BSpline控制点
    void generateBspline(Eigen::Vector3d curpos, const std::vector<Eigen::Vector3d> &controlPoints,
                         int degree, double time, double yaw,int num);
};