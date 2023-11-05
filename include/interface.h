#include <boost/sml.hpp>
#include "ros/ros.h"

#include "mavros_msgs/CommandSetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTakeoffLocal.h"
#include "mavros_msgs/SetTFListen.h"
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

namespace sml = boost::sml;
using namespace std;

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
        void SetPosition_yaml(tf::Vector3 position_set)
        {
            position = position_set;
            rotation = tf::Quaternion(0, 0, 0, 1);
        }
        void SetSpeed(double x, double y, double z)
        {
            vsT.coordinate_frame = 1;
            vsT.type_mask=4039+24;
            //vsT_body_axis.type_mask=4039+24;
            vsT.velocity.x = x;
            vsT.velocity.y = y;
            vsT.velocity.z = z;
        }

    };
    struct interface
    {
        mavros_msgs::State current_state;
        linetracing::CustomMsg cu_msg;
        ros::NodeHandle ros_handle;
        ros::Subscriber state_sub = ros_handle.subscribe<mavros_msgs::State>("mavros/state", 10, [&](const mavros_msgs::State::ConstPtr &msg)   
                                                                    { current_state = *msg; });
        ros::Publisher vel_sp_pub = ros_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ros::Publisher target_pub;
        ros::ServiceClient client;
        tf::TransformBroadcaster broadcaster;
        tf::TransformListener listener;
        tf::StampedTransform transform;
        TFtarget _target;
        
        //pid ���ٶ�
        mavros_msgs::PositionTarget vsT_pid;
        PIDController mypid_x;
        PIDController mypid_y;
        //PIDController mypid_z;
        //PIDController mypid_z_angular;
        YAML::Node message1 = YAML::LoadFile("/home/drone/acfly_ws/src/linetracing/config/msg.yaml");
        double coff = message1["coff"].as<double>();

        void Set2Offboard()
        {
            mavros_msgs::CommandSetMode offb_set_mode;
            offb_set_mode.request.base_mode = 0;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            client = ros_handle.serviceClient<mavros_msgs::CommandSetMode>("/mavros/cmd/set_mode");
            client.call(offb_set_mode);
        }

        void Set2Armed(bool open)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = open;
            client = ros_handle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
            client.call(arm_cmd);
        }

        void openTFlisten(bool open)
        {
            client = ros_handle.serviceClient<mavros_msgs::SetTFListen>("/mavros/setpoint_position/set_tf_listen");
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
}