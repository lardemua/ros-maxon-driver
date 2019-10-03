#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "mx_des.h"


const std::vector<float> multipliers{500, 1000, 1500, 2000, 2500, 3000};

struct CarState
{
    bool reversed = false;
    float speed = 0.0;
    float multiplier = 0;
};

class TeleopMotion
{
public:
    TeleopMotion();

private:
    ros::NodeHandle nh_;
    ros::Subscriber enable_sub_;
    ros::Subscriber reset_sub_;
    ros::Subscriber velocity_;
    ros::Subscriber stop_sub_;

    des_context *mx;
    CarState car_state;
    void velocityCallback(const std_msgs::Float64::ConstPtr &velocity);
    void stopCallback(const std_msgs::Bool::ConstPtr &var);
    void enableCallback(const std_msgs::Bool::ConstPtr &var);
    void resetCallback(const std_msgs::Bool::ConstPtr &var);
    void updateSpeed();
};

TeleopMotion::TeleopMotion()
{
    velocity_ = nh_.subscribe<std_msgs::Float64>("teleopmain/cmd_velocity", 10, &TeleopMotion::velocityCallback, this);
    stop_sub_ = nh_.subscribe<std_msgs::Bool>("teleopmain/cmd_stop", 10, &TeleopMotion::stopCallback, this);
    enable_sub_ = nh_.subscribe<std_msgs::Bool>("teleopmain/cmd_enable", 10, &TeleopMotion::enableCallback, this);
    reset_sub_ = nh_.subscribe<std_msgs::Bool>("teleopmain/cmd_reset", 10, &TeleopMotion::resetCallback, this);

    mx = des_init((char *)"/dev/ttyUSB1", NULL); // eletric motor communication
    // servo motor communication
    if (!mx)
    {
        throw std::runtime_error("error initializing des");
    }
    des_reset(mx);
}

void TeleopMotion::enableCallback(const std_msgs::Bool::ConstPtr &var)
{
    ROS_INFO("enabled");
    des_enable(mx, true);
}

void TeleopMotion::resetCallback(const std_msgs::Bool::ConstPtr &var)
{
    ROS_INFO("reset");
    des_reset(mx);
}

void TeleopMotion::velocityCallback(const std_msgs::Float64::ConstPtr &velocity)
{
    ROS_INFO("speed = %d", (short)velocity->data);
    des_set_velocity(mx, (short)velocity->data);
}

void TeleopMotion::stopCallback(const std_msgs::Bool::ConstPtr &var)
{
    ROS_INFO("motor stopped");
    des_stop_motion(mx);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_motion_node");
    TeleopMotion teleop_motion_atlas;
    ros::spin();
}
