#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>

class TeleopSteering
{
public:
    TeleopSteering();

private:
    ros::NodeHandle nh_;
    ros::Subscriber steering_sub_;
    ros::Subscriber reset_sub_;
    ros::Publisher steering_pub_;

    void steeringCallback(const std_msgs::UInt16::ConstPtr &steering);
    void resetCallback(const std_msgs::Bool::ConstPtr &reset);
};

TeleopSteering::TeleopSteering()
{

    steering_sub_ = nh_.subscribe<std_msgs::UInt16>("teleop_node/cmd_steering", 10, &TeleopSteering::steeringCallback, this);
    reset_sub_ = nh_.subscribe<std_msgs::Bool>("teleop_node/cmd_reset", 10, &TeleopSteering::resetCallback, this);
    steering_pub_ = nh_.advertise<std_msgs::UInt16>("teleop_node/cmd_steering", 10);
}

void TeleopSteering::resetCallback(const std_msgs::Bool::ConstPtr &reset)
{
    ROS_INFO("steering reset.");
}

void TeleopSteering::steeringCallback(const std_msgs::UInt16::ConstPtr &steering)
{
    int angle = steering->data;
    ROS_INFO("steering angle: %d", angle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_steering_node");
    TeleopSteering teleop_steering_atlas;
    ros::spin();
}