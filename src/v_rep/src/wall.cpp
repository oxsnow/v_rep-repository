#include "ros/ros.h"
#include "math.h"
#include "angles/angles.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"

#define x_1 148
#define y_1 -150
#define x_2 -175
#define y_2 -150
#define x_3 -200
#define y_3 175
#define x_4 75
#define y_4 100

ros::Timer interupt_move;
ros::Timer tim_motor;

ros::Publisher pub_velocity;

ros::Subscriber sub_keyboard;
ros::Subscriber sub_sensor;
ros::Subscriber sub_orientation;

float pos_x, pos_y, theta;
float sensor[17];
float x_point[4] = {148, -175, -200, 75};
float y_point[4] = {-150, -150, 175, 100};

double time_delay;

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
    float raw_motor_left, raw_motor_right, temp_motor_left, temp_motor_right, motor_left, motor_right;
    float v, w;
    float l_acc, r_acc;
    float r = 19.5 / 2, l = 38.1;
    raw_motor_right = _v / r + _w * l / r;
    raw_motor_left = _v / r - _w * l / r;

    temp_motor_left = raw_motor_left;
    temp_motor_right = raw_motor_right;

    l_acc = (raw_motor_left - temp_motor_left) / 100;
    r_acc = (raw_motor_right - temp_motor_right) / 100;

    motor_left += l_acc;
    motor_right += r_acc;

    std_msgs::Float32MultiArray msg_motor;
    msg_motor.data.push_back(raw_motor_left);
    msg_motor.data.push_back(raw_motor_right);

    pub_velocity.publish(msg_motor);
}

void wall_following(int _follow_index, float _v, float _w)
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
        if (avr > 0.5 && avr < 0.8 && sensor[5] < 0.8)
        {
            move(_v, 0);
            counter = count_r = count_l = 0;
        }

        //left
        else if (avr > 0.8 || sensor[5] > 0.8)
        {
            move(0, _w);
            count_r = 1;

            if (count_l)
                counter++;

            count_l = 0;
        }

        //right
        else if (avr < 0.5)
        {
            move(0, -_w);
            count_l = 1;
            if (count_r)
                counter++;
            count_r = 0;
        }

        //failsafe
        if (counter > 2)
        {
            move(_v, 0);
            counter = 0;
        }
        status_follow = 0;
        break;

    case 2:
        avr = sensor[0];

        //forward
        if (avr > 0.5 && avr < 0.8 && sensor[2] < 0.8)
        {
            move(_v, 0);
            counter = count_r = count_l = 0;
        }

        //right
        else if (avr > 0.8 || sensor[2] > 0.8)
        {
            move(0, -_w);
            count_l = 1;

            if (count_r)
                counter++;

            count_r = 0;
        }

        //left
        else if (avr < 0.5)
        {
            move(0, _w);
            count_r = 1;

            if (count_l)
                counter++;

            count_l = 0;
        }

        //failsafe
        if (counter > 2)
        {
            move(_v, 0);
            counter = 0;
        }
        status_follow = 0;
        break;
    }
}

void movement(float _v, float _w)
{
    static int status_case;

    static float temp_pos_x, temp_pos_y;
    static bool move_status, pos_status;
    static int room, home;
    static float room_dist[4];
    static float helper = 1000;

    switch (status_case)
    {
    case 0:
        temp_pos_x = pos_x;
        temp_pos_y = pos_y;

        for (int i = 0; i < 4; i++)
        {
            room_dist[i] = dist(temp_pos_x, temp_pos_y, x_point[i], y_point[i]);
        }
        for (int i = 0; i < 4; i++)
        {
            if (room_dist[i] < helper)
            {
                helper = room_dist[i];
                room = home = i + 1;
            }
        }
        status_case = 1;
        break;

    case 1:
        if (pos_x > 0 && pos_x < 150 && pos_y > 55)
            wall_following(0, _v, _w);
        else
            wall_following(1, _v, _w);

        status_case = 2;
        break;

    case 2:

        if(!move_status && dist(pos_x, pos_y, x_point[home - 1], y_point[home - 1]) < 100)
            pos_status = false;
        else
            pos_status = true;
        
        if (dist(pos_x, pos_y, x_1, y_1) < 50 && pos_status)
        {
            move_status = true;
            room = 1;
        }
        if (dist(pos_x, pos_y, x_2, y_2) < 50 && pos_status)
        {
            move_status = true;
            room = 2;
        }
        if (dist(pos_x, pos_y, x_3, y_3) < 50 && pos_status)
        {
            move_status = true;
            room = 3;
        }
        if (dist(pos_x, pos_y, x_4, y_4) < 30 && pos_status)
        {
            move_status = true;
            room = 4;
        }

        if (move_status && room == home)
            status_case = 3;
        else
            status_case = 1;

        break;

    case 3:
        ROS_INFO("mission complete");
        move(0, 0);
        break;
    }

    ROS_INFO("index %d %d %d %d %.2f %.2f %.2f", room, home, pos_status, move_status, dist(pos_x, pos_y, x_point[home - 1], y_point[home - 1]), sensor[0], sensor[7]);
}

void cllbck_interupt_move(const ros::TimerEvent &event)
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
    if (ros::Time::now().toSec() - time_delay > 1)
    {
        movement(15, 0.15);
    }
}

void cllbck_tim_motor(const ros::TimerEvent &event)
{
}

void cllbck_sub_keyboard(const geometry_msgs::TwistConstPtr &msg)
{
    float v,w;
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
    ros::init(argc, argv, "wall");

    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    time_delay = ros::Time::now().toSec();

    interupt_move = NH.createTimer(ros::Duration(0.01), cllbck_interupt_move);

    pub_velocity = NH.advertise<std_msgs::Float32MultiArray>("velocity_array", 16);

    sub_keyboard = NH.subscribe("turtle1/cmd_vel", 16, cllbck_sub_keyboard);
    sub_sensor = NH.subscribe("sensor_array", 16, cllbck_sub_sensor);
    sub_orientation = NH.subscribe("orientation", 16, cllbck_sub_orientation);

    MTS.spin();
}
