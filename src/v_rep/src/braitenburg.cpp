#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

ros::Timer tim_50hz;

ros::Publisher pub_velocity;

ros::Subscriber sub_sensor;

float motor_left, motor_right;
float sensor[16];

void cllbck_50hz(const ros::TimerEvent &event)
{
    std_msgs::Int8MultiArray msg_motor;

    motor_left = motor_left + 0.02;
    motor_right = motor_right + 0.02;

    msg_motor.data.push_back(motor_left);
    msg_motor.data.push_back(motor_right);

    pub_velocity.publish(msg_motor);

    for(int i = 0; i < 16; i++)
    ROS_INFO("%d", sensor[i]);
}

void cllbck_sub_sensor(std_msgs::Float32MultiArrayConstPtr &msg)
{
    for(int i = 0; i < 16; i++)
    {    
        sensor[i] = msg->data[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "routine");
    
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_50hz);

    pub_velocity = NH.advertise<std_msgs::Int8MultiArray>("velocity_array",16);

    sub_sensor = NH.subscribe("sensor_array",16, cllbck_sub_sensor);

    MTS.spin();
}
