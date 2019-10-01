#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "mx_des.h"

class TeleopAtlasMV
{
public:
    TeleopAtlasMV();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    ros::NodeHandle nh_;
    int linear_, angular_;
    double l_scale_, a_scale_;
    ros::Publisher cmd_pub_;
    ros::Subscriber joy_sub_;

    des_context *mx;
};

TeleopAtlasMV::TeleopAtlasMV() : linear_(1), angular_(3), l_scale_(0.025), a_scale_(0.025)
{
    // nh_.param("axis_linear", linear_, linear_);
    // nh_.param("axis_angular", angular_, angular_);
    // nh_.param("scale_angular", a_scale_, a_scale_);
    // nh_.param("scale_linear", l_scale_, l_scale_);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("atlasmv_commands", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopAtlasMV::joyCallback, this);

    mx = des_init((char *)"/dev/ttyUSB1", NULL);
    if (!mx)
    {
        throw std::runtime_error("error initializing des");
    }
}

void TeleopAtlasMV::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

    if (joy->buttons[7] == 1) //command "enable"-->button: START
    {

        des_error err = des_enable(mx, true);
        if (err != DES_OK)
        {
            ROS_ERROR("enabling %s", des_strerror(err));
        }
        else
        {
            ROS_INFO("enabled");
        }
    }
    if (joy->buttons[2] == 1) //commmand "STOP"-->button: X
    {
        des_error err = des_stop_motion(mx);
        if (err != DES_OK)
        {
            ROS_ERROR("stopping %s", des_strerror(err));
        }
        else
        {
            ROS_INFO("motor stopped");
        }
    }
    if (joy->buttons[6] == 1) //command "reset"-->button: BACK
    {
        des_error err = des_reset(mx);
        if (err != DES_OK)
        {
            ROS_ERROR("reseting %s", des_strerror(err));
        }
        else
        {
            ROS_INFO("reset done");
        }
    }
    //command "speed":--->button RT
    float speed_axes = joy->axes[5];
    int max_velocity = 2000;
    short speed = ((-speed_axes + 1.0) / 2.0) * max_velocity;
    des_set_velocity(mx, speed);
    // ROS_INFO("Setting speed to %d", speed);
    std::cout << "[SET]Setting speed to " << speed << std::endl;

    //command "read speed"
    des_reading act_speed;
    des_read_velocity(mx, false, &act_speed);
    // ROS_INFO("Velocity read %d", act_speed.real);
    std::cout << "[READ]Speed read " << act_speed.real << std::endl;

    int incr = 250;
    if ((joy->buttons[5] == 1) && (joy->buttons[4] == 0) && (act_speed.real < max_velocity - incr)) // inccrease velocity rate
    {
        if (act_speed.real < incr)
        {
            des_set_velocity(mx, max_velocity / 2);
        }
        else
        {
            des_set_velocity(mx, act_speed.real + incr);
            ROS_INFO("Inc speed in +%d", incr);
        }
    }

    if (joy->buttons[4] == 1 && act_speed.real > incr && joy->buttons[5] == 0) //decrease velocity rate
    {
        des_set_velocity(mx, act_speed.real - incr);
        ROS_INFO("Dec speed in -%d", incr);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_atlasMV_node");
    TeleopAtlasMV teleop_atlas;
    ros::spin();
}