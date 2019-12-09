#ifndef _TPV_CORE_H_
#define _TPV_CORE_H_

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

// ros message type
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

// ros
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define VECTOR std::vector

// you will use OpenCL for this?
#ifndef TPV_USE_OPENCL
#define TPV_CV_MAT cv::Mat
#else
#define TPV_CV_MAT cv::UMat
#endif

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

#define BLUR_KER_SIZE 3
#define MORPH_ELLIPSE_KER_SIZE 3, 3

#define CANNY_EDGES 30
#define CANNY_THRESHOLD 150

#define DISTANCE_CALC_BIN_THRESHOLD 15

// car driving const
#define SET_STEER_API_HOOK "/set_steer_car_api"
#define SET_SPEED_API_HOOK "/set_speed_car_api"
#define DEFAULT_QUEUE_SIZE 10
#define FLOAT_MSG_TYPE std_msgs::Float32

#define ERR_THRESHOLD 3

#endif