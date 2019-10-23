#ifndef ROUTINE_H_
#define ROUTINE_H_

#include "ros/ros.h"
#include "math.h"
#include "angles/angles.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"

ros::Timer interupt;
ros::Timer tim_motor;

ros::Publisher pub_velocity;

ros::Subscriber sub_keyboard;
ros::Subscriber sub_sensor;
ros::Subscriber sub_orientation;

float pos_x, pos_y, theta;
float sensor[16];

float motor_left, motor_right;

#endif