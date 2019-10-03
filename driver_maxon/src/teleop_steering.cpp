#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

class TeleopSteering
{
public:
    TeleopSteering();

private:
    ros::NodeHandle nh_;
    ros::Subscriber steering_sub_;

    void steeringCallback(const std_msgs::Float64::ConstPtr &steering);
};

TeleopSteering::TeleopSteering()
{
    steering_sub_ = nh_.subscribe<std_msgs::Float64>("teleop_node/cmd_steering", 10, &TeleopSteering::steeringCallback, this);
}

void TeleopSteering::steeringCallback(const std_msgs::Float64::ConstPtr &steering)
{
    float angle = steering->data;
    ROS_INFO("steering angle:%f",angle);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_steering_node");
    TeleopSteering teleop_steering_atlas;
    ros::spin();
}