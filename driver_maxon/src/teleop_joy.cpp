#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "mx_des.h"

struct ButtonState
{
    bool enable = false;
    bool stop_motion = false;
    float speed = 0.0;
    bool reverse = false;
    bool reset = false;
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

    return state;
}

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
    ButtonState btn_state;
    CarState car_state;
};

TeleopAtlasMV::TeleopAtlasMV() : linear_(1), angular_(3), l_scale_(0.025), a_scale_(0.025)
{

    mx = des_init((char *)"/dev/ttyUSB1", NULL);
    if (!mx)
    {
        throw std::runtime_error("error initializing des");
    }

    des_reset(mx);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("atlasmv_commands", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopAtlasMV::joyCallback, this);
}

void TeleopAtlasMV::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

    const ButtonState next_state = from_joy(*joy);

    // std::cout
    //     << "button state { "
    //     << "speed = " << next_state.speed << " }"
    //     << std::endl;

    if (next_state.enable && !btn_state.enable)
    {
        des_enable(mx, true);
    }

    if (next_state.reset && !btn_state.reset)
    {
        des_reset(mx);
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

    if (update_speed)
    {
        bool flip = true;
        bool reversed = car_state.reversed;
        float normalized_speed = car_state.speed;
        float multiplier = multipliers[car_state.multiplier];

        std::cout << "normalized_speed = " << normalized_speed << std::endl;

        float speed = (reversed ^ flip ? -1 : 1) * normalized_speed * multiplier;

        ROS_INFO("speed = %d", (short)speed);

        des_set_velocity(mx, (short)speed);
    }

    btn_state = next_state;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_atlasMV_node");
    TeleopAtlasMV teleop_atlas;
    ros::spin();
}