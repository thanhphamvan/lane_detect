#include "carcontrol.h"

CarControl::CarControl()
{
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("Team1_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("Team1_speed",50);
}

CarControl::~CarControl() {}

void CarControl::driverCar(float error, bool changeDir)
{
    float velocity = maxVelocity;
    if (changeDir)
    {
        velocity -= 20;
    }

    if (abs(error) > 15)
    {
        velocity -= error - 15;
    }

    if (velocity < minVelocity)
    {
        velocity = minVelocity;
    }

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 
