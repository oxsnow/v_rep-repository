#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"

ros::Timer interupt;

ros::Publisher pub_velocity;

ros::Subscriber sub_sensor;
ros::Subscriber sub_orientation;

float pos_x, pos_y, theta;
float sensor[16];


void cllbck_interupt(const ros::TimerEvent &event)
{
    
}

void cllbck_sub_sensor(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    for(int i = 0; i < 16; i++)
        sensor[i] = msg->data[i];
}

void cllbck_sub_orientation(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    pos_x = msg->data[0];
    pos_y = msg->data[1];
    theta = msg->data[2]; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hub");

    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    interupt = NH.createTimer(ros::Duration(0.1), cllbck_interupt);

    pub_velocity = NH.advertise<std_msgs::Float32MultiArray>("velocity_array",16);

    sub_sensor = NH.subscribe("sensor_arary", 16, cllbck_sub_sensor);
    sub_orientation = NH.subscribe("orientation", 16, cllbck_sub_orientation);

    MTS.spin();
}
