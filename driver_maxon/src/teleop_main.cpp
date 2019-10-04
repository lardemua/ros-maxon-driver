#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include "mx_des.h"

struct ButtonState
{
    bool enable = false;
    bool stop_motion = false;
    float speed = 0.0;
    float steering_lr = 0.0;
    bool reverse = false;
    bool reset = false;
    bool const_speed_up = false;
    bool const_speed_dn = false;
    bool multiplier_increment = false;
    bool multiplier_decrement = false;
};

const std::vector<float> multipliers{500, 1000, 1500, 2000, 2500, 3000};

struct CarState
{
    bool reversed = false;
    float speed = 0.0;
    float multiplier = 0;
};

ButtonState from_joy(const sensor_msgs::Joy &joy)
{
    ButtonState state;

    state.enable = joy.buttons[7] == 1;
    state.stop_motion = joy.buttons[2] == 1;
    state.reverse = joy.buttons[1] == 1;
    state.reset = joy.buttons[6] == 1;

    state.multiplier_increment = joy.buttons[5] == 1;
    state.multiplier_decrement = joy.buttons[4] == 1;

    state.speed = (((-joy.axes[5] + 1.0) / 2.0));
    state.steering_lr = ((joy.axes[0] + 1) / 2.0);
    state.const_speed_up = joy.buttons[3] == 1;
    state.const_speed_dn = joy.buttons[0] == 1;

    return state;
}
class TeleopMain
{
public:
    TeleopMain();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Publisher enable_pub_;
    ros::Publisher reset_pub_;
    ros::Publisher velocity_pub_;
    ros::Publisher stop_pub_;
    ros::Publisher steering_pub_;

    des_context *mx;
    ButtonState btn_state;
    CarState car_state;
};

TeleopMain::TeleopMain()
{

    // des_reset(mx);
    //Bool topics:
    enable_pub_ = nh_.advertise<std_msgs::Bool>("teleop_node/cmd_enable", 10);
    reset_pub_ = nh_.advertise<std_msgs::Bool>("teleop_node/cmd_reset", 10);
    stop_pub_ = nh_.advertise<std_msgs::Bool>("teleop_node/cmd_stop", 10);

    //Float topics:
    velocity_pub_ = nh_.advertise<std_msgs::Float64>("teleop_node/cmd_velocity", 10);
    steering_pub_ = nh_.advertise<std_msgs::UInt16>("teleop_node/cmd_steering", 10);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMain::joyCallback, this);
}

void TeleopMain::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    const ButtonState next_state = from_joy(*joy);
    std_msgs::Bool enable;
    std_msgs::Bool reset;
    std_msgs::Bool stop;
    std_msgs::Float64 msg_speed;
    std_msgs::UInt16 msg_steering;

    if (next_state.enable && !btn_state.enable)
    {
        enable.data = next_state.enable;
        enable_pub_.publish(enable);
    }
    if (next_state.reset && !btn_state.reset)
    {
        reset.data = next_state.reset;
        reset_pub_.publish(reset);
    }
    if (next_state.stop_motion && !btn_state.stop_motion)
    {
        stop.data = next_state.stop_motion;
        stop_pub_.publish(stop);
    }
    bool update_speed = false;
    if (next_state.multiplier_increment && !btn_state.multiplier_increment)
    {
        car_state.multiplier += car_state.multiplier < multipliers.size();
        update_speed = true;
    }
    if (next_state.multiplier_decrement && !btn_state.multiplier_decrement)
    {
        car_state.multiplier -= car_state.multiplier > 0;
        update_speed = true;
    }

    if (next_state.speed != btn_state.speed)
    {
        car_state.speed = next_state.speed > 0.05 ? next_state.speed : 0.0;

        std::cout << "new speed " << car_state.speed << std::endl;
        update_speed = true;
    }

    if (next_state.reverse != btn_state.reverse)
    {
        car_state.reversed = next_state.reverse;
        update_speed = true;
    }
    int inc = car_state.multiplier / 4;
    if (next_state.const_speed_up && !btn_state.const_speed_up)
    {
        if (car_state.speed == 0)
        {
            car_state.speed = 0.25;
            update_speed = true;
        }
        else
        {
            car_state.speed = car_state.speed * 1.25;
            update_speed = true;
        }
    }
    if (next_state.const_speed_dn && !btn_state.const_speed_dn)
    {
        car_state.speed = car_state.speed * 0.75;
        update_speed = true;
    }

    if (update_speed)
    {
        bool flip = true;
        bool reversed = car_state.reversed;
        float normalized_speed = car_state.speed;
        float multiplier = multipliers[car_state.multiplier];

        std::cout << "normalized_speed = " << normalized_speed << std::endl;

        float speed = (reversed ^ flip ? -1 : 1) * normalized_speed * multiplier;
        msg_speed.data = speed;

        velocity_pub_.publish(msg_speed);
    }

    uint16_t multiplier_steering = 180;
    msg_steering.data = (180-(multiplier_steering * next_state.steering_lr));
    if (msg_steering.data >= 170)
    {
        msg_steering.data = 160;
    }
    else if (msg_steering.data <= 10)
    {
        msg_steering.data = 20;
    }

    steering_pub_.publish(msg_steering);
    btn_state = next_state;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "teleop_node");
    TeleopMain teleop_main_atlas;

    ros::spin();
}