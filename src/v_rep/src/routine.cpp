#include "ros/ros.h"
#include "math.h"
#include "angles/angles.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"

ros::Timer interupt_keyboard;
ros::Timer tim_motor;

ros::Publisher pub_velocity;

ros::Subscriber sub_keyboard;
ros::Subscriber sub_sensor;
ros::Subscriber sub_orientation;

float pos_x, pos_y, theta;
float sensor[17];

float motor_left, motor_right;
float v, w;
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

bool wall_following(int _follow_index)
{
    static int status_follow;
    static float avr;
    static int counter;
    static bool count_l, count_r;
    static double time;
    switch (status_follow)
    {
    case 0:
        switch (_follow_index)
        {
        case 0:
            status_follow = 1;
            break;
        case 1:
            status_follow = 2;
            break;
        }
        break;

    case 1:
        avr = sensor[7];
        //forward
        if(avr > 0.8 && avr < 0.9 && sensor[5] < 0.8)
        {
            move(10,0);
            counter = count_r = count_l = 0;
        }

        //left
        else if(avr > 0.9 || sensor[5] > 0.8)
        {
            move(0,0.10);
            count_r = 1;

            if (count_l) 
                counter++;

            count_l = 0;
        }

        //right
        else if(avr < 0.8)
        {
            move(0,-0.10);
            count_l = 1;

            if(count_r)
                counter++;
            
            count_r = 0;
        }

        //failsafe
        if(counter > 2)
        {
            move(10,0);
            counter = 0;
        }
        // status_follow = 0;
        if(sensor[7] < 0.01) status_follow = 3;
        ROS_INFO("sensor7 %f", sensor[7]);
        time = ros::Time::now().toSec();
        break;

    case 2:
        avr = sensor[0];
        
        //forward
        if(avr > 0.8 && avr < 0.9 && sensor[2] < 0.8)
        {
            move(10,0);
            counter = count_r = count_l = 0;
        }

        //right
        else if(avr > 0.9 || sensor[2] > 0.8)
        {
            move(0,-0.10);
            count_l = 1;

            if (count_r) 
                counter++;

            count_r = 0;
        }

        //left
        else if(avr < 0.8)
        {
            move(0,0.10);
            count_r = 1;

            if(count_l)
                counter++;
            
            count_l = 0;
        }

        //failsafe
        if(counter > 2)
        {
            move(10,0);
            counter = 0;
        }
        // status_follow = 0;

        if(sensor[0] < 0.5 && sensor[1] < 0.5) status_follow = 3;
        time = ros::Time::now().toSec();
        ROS_INFO("sensor0 %f", sensor[0]); 
        break;

        case 3:
        move(10,0);
        if(ros::Time::now().toSec() - time > 2) status_follow = 99;
        break;
    }

    ROS_INFO("index %d %f %f", status_follow, sensor[0], sensor[7]);

    if(status_follow == 99)
    {
        status_follow = 0;
        return true;
    }
    else return false;
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

void avoid(float _v, float _w)
{
    float r = 19.5 / 2, l = 38.1;
    float braitenbergL[16] = {0, -0.4, -0.6, -0.8, -1, -1.2, -1.4, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float braitenbergR[16] = {0, -1.4, -1.2, -1, -0.8, -0.6, -0.4, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    motor_right = _v / r + _w * l / r;
    motor_left = _v / r - _w * l / r;

    for (int i = 0; i < 16; i++)
    {
        motor_right += braitenbergR[i] * sensor[i];
        motor_left += braitenbergL[i] * sensor[i];
    }

    std_msgs::Float32MultiArray msg_motor;
    msg_motor.data.push_back(motor_left);
    msg_motor.data.push_back(motor_right);

    pub_velocity.publish(msg_motor);

    ROS_INFO("avoid %f %f %f", motor_left, motor_right, braitenbergR[5]);
}

void move_position(float _v, float _w, float _x, float _y)
{
    int ang_target = angle(_x, _y);
    int pos_target = dist(pos_x, pos_y, _x, _y);
    bool status, temp_status;
    static double time;
    static int hysterisis_in, hysterisis_out;
    static int case_status, temp_case_status;

    float obs_angle = abs(ang_target - 90 - theta);
    int sensor_index = obs_angle / 22.5;
    if (sensor_index > 7 || sensor_index < 0)
        sensor_index = 16;

    if(hysterisis_in > 10)
        temp_status = status = true;
    else if (hysterisis_out > 10)
        temp_status = status = false;

    switch (case_status)
    {
        case 0:
        if (sensor[sensor_index] > 0.8) case_status = temp_case_status = 1;
        else case_status = temp_case_status = 2;
        if(sensor_index == 16) case_status = temp_case_status;
        break;

        case 1:
        if (sensor_index >= 0 && sensor_index <= 3)
        {
            if(wall_following(1)) case_status = 2;
        }
        else if (sensor_index >= 4 && sensor_index <= 7)
        {
            if(wall_following(0)) case_status = 2;
        }
        else case_status = 0;
        break;

        case 2:
        if (abs(theta - ang_target) > 5)
        {
                if (theta - ang_target > 0)
                    w = -_w;
                if (theta - ang_target < 0)
                    w = _w;
        }
        else
            w = 0;

        if (abs(pos_target) > 20 && abs(theta - ang_target) < 10)
        v = _v;
        else v = 0;
        move(v,w);
        case_status = 0;
        break;
    }

    // ROS_INFO("sensor %d %d %f %f %f", case_status, sensor_index, sensor[sensor_index], w, v);
}


void cllbck_interupt_keyboard(const ros::TimerEvent &event)
{
    // std_msgs::Float32MultiArray msg_motor;
    // if(motor_left > 0)
    //     motor_left -= 0.1;
    // if(motor_right > 0)
    //     motor_right -= 0.1;
    // if(motor_left < 0)
    //     motor_left += 0.1;
    // if(motor_right < 0)
    //     motor_right += 0.1;
    // motor_left = motor_right = 0;
    // msg_motor.data.push_back(motor_left);
    // msg_motor.data.push_back(motor_right);

    // pub_velocity.publish(msg_motor);

    move_position(10, 0.1, 0, 0);
    // wall_following(0);

    // ROS_INFO("%d %f", sensor_index, sensor[sensor_index]);
}

void cllbck_sub_keyboard(const geometry_msgs::TwistConstPtr &msg)
{
    v = msg->linear.x * 10;
    w = msg->angular.z / 4;

    // ROS_INFO("L: %f  R:%f", motor_left, motor_right);
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

    sub_keyboard = NH.subscribe("turtle1/cmd_vel", 16, cllbck_sub_keyboard);
    sub_sensor = NH.subscribe("sensor_array", 16, cllbck_sub_sensor);
    sub_orientation = NH.subscribe("orientation", 16, cllbck_sub_orientation);

    MTS.spin();
}
