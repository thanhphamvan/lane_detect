#ifndef _TPV_CORE_H_
#define _TPV_CORE_H_

#include <vector>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

// ros message type
#include <std_msgs/Float32.h>

#define VECTOR std::vector

// lane detecting const
#define MIN_THRESHOLD {0, 0, 170}
#define MAX_THRESHOLD {179, 30, 255}

#define BINARY_THRESHOLD 180
#define VERTICAL_SKY_LINE 120
#define LANE_WIDTH 120

#define STATIC_WIDTH 320
#define STATIC_HEIGHT 240

#define MIN_VELOCITY 30
#define MAX_VELOCITY 60

// car driving const
#define SET_STEER_API_HOOK "/set_steer_car_api"
#define SET_SPEED_API_HOOK "/set_speed_car_api"
#define DEFAULT_QUEUE_SIZE 10
#define FLOAT_MSG_TYPE std_msgs::Float32

#define ERR_THRESHOLD 3

#endif