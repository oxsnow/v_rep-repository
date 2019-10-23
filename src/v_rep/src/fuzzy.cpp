#include "ros/ros.h"
#include "math.h"
#include "angles/angles.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"

ros::Timer interupt_keyboard;

ros::Publisher pub_velocity;

ros::Subscriber sub_keyboard;
ros::Subscriber sub_sensor;
ros::Subscriber sub_orientation;

float pos_x, pos_y, theta;
float sensor[17];

float motor_left, motor_right;
void move(float _v, float _w);
float dist(float _pos_x1, float _pos_y1, float _pos_x2, float _pos_y2)
{
    float delta_x = _pos_x1 - _pos_x2;
    float delta_y = _pos_y1 - _pos_y2;

    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

float angle(float _target_x, float _target_y)
{
    float delta_x = _target_x - pos_x;
    float delta_y = _target_y - pos_y;

    return angles::to_degrees(atan2(delta_y, delta_x));
}



void move(float _v, float _w)
{
    float r = 19.5 / 2, l = 38.1;
    motor_right = _v / r + _w * l / r;
    motor_left = _v / r - _w * l / r;

    std_msgs::Float32MultiArray msg_motor;
    msg_motor.data.push_back(motor_left);
    msg_motor.data.push_back(motor_right);

    pub_velocity.publish(msg_motor);
}




void cllbck_interupt_keyboard(const ros::TimerEvent &event)
{
}


void cllbck_sub_sensor(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    for (int i = 0; i < 16; i++)
        sensor[i] = msg->data[i];
    sensor[17] = 0;
    // ROS_INFO("sensor %f %f %f %f", sensor[0], sensor[1], sensor[2], sensor[3]);
}

void cllbck_sub_orientation(const std_msgs::Float32MultiArrayConstPtr &msg)
{
    pos_x = msg->data[0];
    pos_y = msg->data[1];
    theta = msg->data[2];

    while (theta > 180)
        theta -= 360;
    while (theta < -180)
        theta += 360;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "routine");

    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    interupt_keyboard = NH.createTimer(ros::Duration(0.1), cllbck_interupt_keyboard);

    pub_velocity = NH.advertise<std_msgs::Float32MultiArray>("velocity_array", 16);

    sub_sensor = NH.subscribe("sensor_array", 16, cllbck_sub_sensor);
    sub_orientation = NH.subscribe("orientation", 16, cllbck_sub_orientation);

    MTS.spin();
}
