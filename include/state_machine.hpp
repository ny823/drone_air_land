#include <boost/sml.hpp>

#include "ros/ros.h"
#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include "mavros_msgs/SetTFListen.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "../include/PID.hpp"
#include "../include/color.hpp"

#include <yaml-cpp/yaml.h>
#include "std_srvs/SetBool.h"

#define SphereDis 0.06

namespace sml = boost::sml;
using namespace std;

void read_PID(const YAML::Node &pid_yaml, PIDController &pid, int i)
{ if (i = 0) {
    pid.Kd = pid_yaml["Kd"].as<double>();
    pid.Ki = pid_yaml["Ki"].as<double>();
    pid.Kp = pid_yaml["Kp"].as<double>();
    pid.limMax = pid_yaml["limMax"].as<double>();
    pid.limMin = pid_yaml["limMin"].as<double>();
    pid.limMaxInt = pid_yaml["limMaxInt"].as<double>();
    pid.limMinInt = pid_yaml["limMinInt"].as<double>();
    pid.T = pid_yaml["T"].as<double>(); }
if (i = 1) {
    pid.Kd = pid_yaml["Kd_"].as<double>();
    pid.Ki = pid_yaml["Ki_"].as<double>();
    pid.Kp = pid_yaml["Kp_"].as<double>();
    pid.limMax = pid_yaml["limMax_"].as<double>();
    pid.limMin = pid_yaml["limMin_"].as<double>();
    pid.limMaxInt = pid_yaml["limMaxInt_"].as<double>();
    pid.limMinInt = pid_yaml["limMinInt_"].as<double>();
    pid.T = pid_yaml["T"].as<double>();} }

namespace drone
{
    struct TFtarget
    {
        tf::Vector3 position = tf::Vector3(0, 0, 0);
        tf::Quaternion rotation = tf::Quaternion(0, 0, 0, 1);
        mavros_msgs::PositionTarget vsT;
        void SetPosition(double x, double y, double z)
        {
            position = tf::Vector3(x, y, z);
            rotation = tf::Quaternion(0, 0, 0, 1);
        }
        void SetTarget(double p, double i, double j, double k, double x, double y, double z)
        {
            position = tf::Vector3(x, y, z);
            rotation = tf::Quaternion(p, i, j, k);
        }
        void SetTarget(double angle, double x, double y, double z)
        {
            geometry_msgs::Quaternion rotate = tf::createQuaternionMsgFromYaw(angle);
            tf::Quaternion tf_rotate; 
            tf::quaternionMsgToTF(rotate, tf_rotate);
            tf::Quaternion new_rotate = tf::Quaternion(0, 0, 0, 1) * tf_rotate;
            position = tf::Vector3(x, y, z);
            rotation = new_rotate.normalize();
        }
        //设置body系目标时，应该先关闭setposition的tf监听，避免冲突
        //设置body系相对目标位置，目标位置连续发布会一直朝一个方向移动
        void SetBodyPosition(double x, double y, double z)
        {
            vsT.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            vsT.type_mask=0b110111111000;
            vsT.position.x = x;
            vsT.position.y = y;
            vsT.position.z = z;
        }
         //设置body系相对位置和速度，目标位置连续发布会一直朝一个方向移动
        void SetBodyPosition_Speed(double x, double y, double z ,double vx, double vy, double vz)
        {
            vsT.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            vsT.type_mask=0b110111000000;
            vsT.position.x = x;
            vsT.position.y = y;
            vsT.position.z = z;
            vsT.velocity.x = vx;
            vsT.velocity.y = vy;
            vsT.velocity.z = vz;
        }
        //设置body系三维速度
        void SetBodySpeed(double vx, double vy, double vz)
        {
            vsT.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            vsT.type_mask=0b110111000111;
            vsT.velocity.x = vx;
            vsT.velocity.y = vy;
            vsT.velocity.z = vz;
        }
        //PID输入相对位置，输出目标速度
        void SetBodySpeed_PID(double x, double y, double z,PIDController &c_x,PIDController &c_y,PIDController &c_z)
        {
            vsT.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            vsT.type_mask=0b110111000111;
            vsT.velocity.x = PIDController_Update(c_x,x,0,1);
            vsT.velocity.y = PIDController_Update(c_y,y,0,1);
            vsT.velocity.z = PIDController_Update(c_z,z,0,1);
        }
        //设置body系三维速度、加速度，不给z向加速度默认为0
        void SetBodySpeed_Accel(double vx, double vy, double vz,double ax, double ay, double az = 0)
        {
            vsT.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            vsT.type_mask=0b110100000111;
            vsT.velocity.x = vx;
            vsT.velocity.y = vy;
            vsT.velocity.z = vz;
            vsT.acceleration_or_force.x = ax;
            vsT.acceleration_or_force.y = ay;
            vsT.acceleration_or_force.z = 0;
        }
        //设置body系三维速度、角速度、加速度，不给z向加速度默认为0
        void SetBodySpeed_YawRate_Accel(double x, double y, double z,double yaw_rate,double ax, double ay ,double az = 0)
        {
            vsT.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            vsT.type_mask=0b010100000111;
            vsT.velocity.x = x;
            vsT.velocity.y = y;
            vsT.velocity.z = z;
            vsT.acceleration_or_force.x = ax;
            vsT.acceleration_or_force.y = ay;
            vsT.acceleration_or_force.z = 0;
            vsT.yaw_rate = yaw_rate;
        }
        //设置body系角速度
        void SetBodyYawRate(double yaw_rate)
        {
            vsT.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            vsT.type_mask=0b010111111111;
            vsT.yaw_rate = yaw_rate;
        }

    };

    struct dependencies
    {
        mavros_msgs::State current_state;
        bool boot_cmd = false;//启动状态检查

        ros::NodeHandle n;
        ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, [&](const mavros_msgs::State::ConstPtr &msg)   
                                                                    { current_state = *msg; });
        ros::Publisher vel_sp_pub = n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ros::ServiceServer bootServ = n.advertiseService("/mission/boot",&dependencies::boot,this);
        ros::ServiceClient client;
        tf::TransformBroadcaster broadcaster;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        TFtarget _target;
        

        PIDController mypid_x;
        PIDController mypid_y;
        PIDController mypid_z;
        //PIDController mypid_z_angular;
        YAML::Node message1 = YAML::LoadFile("/home/drone/acfly_ws/src/fly_test/config/pid.yaml");
        double coff = message1["coff"].as<double>();

        bool boot(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res)
        {
            ROS_INFO("Boot Message recived!");
            boot_cmd = req.data;
            res.success = req.data;
            if (req.data)
            {res.message = "Mission ON!";}
            else
            {res.message = "Mission Off!";}
            return true;
        };
        void Set2Offboard()
        {
            mavros_msgs::CommandSetMode offb_set_mode;
            offb_set_mode.request.base_mode = 0;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            client = n.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
            client.call(offb_set_mode);
        }

        void Set2Armed(bool open)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = open;
            client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
            client.call(arm_cmd);
        }

        
        void openTFlisten(bool open)
        {
            client = n.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
            mavros_msgs::SetTFListen tf_listen;
            tf_listen.request.value = open;
            client.call(tf_listen);
        }

        void getTransform(std::string target, std::string base)
        {

            try
            {
                listener.lookupTransform(target, base, ros::Time(0), transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }

        bool is_close(tf::Vector3 current_pos, tf::Vector3 target_pos, double threshold)
        {
            tfScalar dis = tf::tfDistance(current_pos, target_pos);
            if (dis <= threshold)
                return true;
            else
                return false;
        }
        bool is_rotated(tf::Quaternion current_quat, tf::Quaternion target_quat, double threshold)
        {
            tfScalar angle = current_quat.angleShortestPath(target_quat);
            if (angle <= threshold)
                return true;
            else
                return false;
        }


    };

    // event
    struct release{};
    struct takeoff{TFtarget target;};
    struct takeoffover{};
    struct land{TFtarget target;};
    struct setspeed{TFtarget target;};
    struct To_lock{};
    struct switchTFmode{bool open;};
    struct moveTo{TFtarget target;};
    struct tickOnce{TFtarget target;};

    // StateMachine
    struct icarus
    {
        auto operator()() const
        {
            using namespace sml;
            // guard
            auto is_init = [](const dependencies &d, ros::Rate _rate)
            {
                if (ros::ok() && d.current_state.connected)
                {
                    return true;
                }
                else
                {
                    ROS_INFO("mavros not connected!!!");
                    return false;
                }
            };
            auto is_boot = [](const dependencies &d)
            {
                return d.boot_cmd;
            };
            auto is_valid = [](dependencies &d, ros::Rate _rate)
            {
                // d.getTransform("camera_init", "base_link");
                // cout<<"-------------------------"<<endl;
                // cout<<"base_link_x: "<<d.transform.getOrigin().x()<<endl;
                // cout<<"base_link_y: "<<d.transform.getOrigin().y()<<endl;
                // cout<<"base_link_z: "<<d.transform.getOrigin().z()<<endl;
                // cout<<"position diff: " <<tf::tfDistance(tf::Vector3(d.transform.getOrigin().x(), d.transform.getOrigin().y(), d.transform.getOrigin().z()), d._target.position) <<endl;
                // cout<<"yaw diff: "<<d.transform.getRotation().angleShortestPath(d._target.rotation) <<endl;
                // cout<<"-------------------------"<<endl;
                if (ros::ok() && d.current_state.armed && d.current_state.mode == "OFFBOARD")
                    return true;
                else
                {
                    // 依次输出错误信息
                    if (!ros::ok())
                    {
                        ROS_INFO("ros not ok");
                    }
                    if (!d.current_state.connected)
                    {
                        ROS_INFO("not connected");
                    }
                    if (!d.current_state.armed)
                    {
                        ROS_INFO("nor armed");
                    }
                    if (d.current_state.mode != "OFFBOARD")
                    {
                        ROS_INFO("not offboard");
                    }
                    ROS_INFO("mode guard faild");
                    return false;
                }
            };

            auto is_near_ground = [](dependencies &d)
            {
                //是否即将触地
                if (d.is_close(tf::Vector3(d.transform.getOrigin().x(), d.transform.getOrigin().y(), d._target.position.z()),
                               d._target.position, 0.08) && d.transform.getOrigin().z()< 0.20 )
                    return true;
                else
                    return false;
            };

            auto is_arrive= [](dependencies &d)
            {
                d.getTransform("camera_init", "base_link");
                cout<<"-------------------------"<<endl;
                cout<<"odom_link_x: "<<d.transform.getOrigin().x()<<endl;
                cout<<"odom_link_y: "<<d.transform.getOrigin().y()<<endl;
                cout<<"odom_link_z: "<<d.transform.getOrigin().z()<<endl;
                cout<<"position diff: " <<tf::tfDistance(tf::Vector3(d.transform.getOrigin().x(), d.transform.getOrigin().y(), d.transform.getOrigin().z()), d._target.position) <<endl;
                cout<<"yaw diff: "<<d.transform.getRotation().angleShortestPath(d._target.rotation) <<endl;
                cout<<"-------------------------"<<endl;
                
                if (d.is_close(tf::Vector3(d.transform.getOrigin().x(), d.transform.getOrigin().y(), d.transform.getOrigin().z()),
                               d._target.position, SphereDis) &&
                    d.is_rotated(d.transform.getRotation(), d._target.rotation, 0.5))
                    return true;
                else
                    return false;
            };

            //action
            auto unlock = [](dependencies &d)
            {
                d.Set2Offboard();
                d.Set2Armed(true);
                d.openTFlisten(true);
                ROS_INFO("Unlocked!");
            };
            auto lock = [](dependencies &d)
            {
                d.Set2Armed(false);
                d.openTFlisten(false);
                ROS_INFO("Locked!");
            };

            auto open_tf = [](dependencies &d)
            {
                d.openTFlisten(true);
                ROS_INFO("TFlisten Quit!");
            };
            auto quit_tf = [](dependencies &d)
            {
                d.openTFlisten(false);
                ROS_INFO("TFlisten Quit!");
            };

            auto setTFtarget = [](dependencies &d)
            {
                d.broadcaster.sendTransform(
                    tf::StampedTransform(tf::Transform(d._target.rotation, d._target.position),ros::Time::now(), "camera_init", "target_position"));
		        ROS_INFO("Set_TF_target");
            };
            auto set_speed = [](dependencies &d)
            {
                d.vel_sp_pub.publish(d._target.vsT);
                ROS_INFO("Set_Speed");
            };

            auto delay = []()
            {
                ros::Duration(1).sleep();
            };

            //在此处或者主函数设置读取PID
            auto set_pid_initial = [](dependencies &d)
            {
            read_PID(d.message1, d.mypid_x, 0);
            read_PID(d.message1, d.mypid_y, 0);
            read_PID(d.message1, d.mypid_z, 1);
            //read_PID(message1, mypid_z_angular, 1);
            PIDController_Init(d.mypid_x);
            PIDController_Init(d.mypid_y);
            PIDController_Init(d.mypid_z);
            };

            return make_transition_table(
                *"idle"_s + event<release>[is_init&&is_boot] / (unlock,delay) = "takingoff"_s,
                "takingoff"_s + event<takeoff>[is_valid] / ([](auto const &e, dependencies &d){ d._target = e.target; },setTFtarget) = "takingoff"_s,
                "takingoff"_s + event<takeoffover>[!is_arrive && is_valid] / setTFtarget ="takingoff"_s,
                "takingoff"_s + event<takeoffover>[is_arrive] / [] {} ="ready"_s,

                "ready"_s + event<moveTo>[is_valid] / ([](auto const &e, dependencies &d){ d._target = e.target; },setTFtarget) = "moving"_s,
                
                "moving"_s + event<moveTo>[is_valid] / ([](auto const &e, dependencies &d){ d._target = e.target; },setTFtarget) = "moving"_s,

                "moving"_s + event<tickOnce>[!is_arrive && is_valid] / setTFtarget = "moving"_s,
                "moving"_s + event<tickOnce>[is_arrive] / [] {} = "ready"_s,
                

                "ready"_s + event<land>[is_valid] / ([](auto const &e, dependencies &d){ d._target = e.target; },setTFtarget) = "landing"_s,
                "landing"_s + event<tickOnce>[ is_near_ground] / lock = X,
                "landing"_s + event<tickOnce>[ !is_near_ground && is_valid] / setTFtarget = "landing"_s,

                "ready"_s + event<To_lock> / [] {} = X

            ); //
        }
    };
};
