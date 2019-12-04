#ifndef _TPV_CORE_H_
#define _TPV_CORE_H_

#include <vector>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#define VECTOR std::vector

#define MIN_THRESHOLD {0, 0, 170}
#define MAX_THRESHOLD {179, 30, 255}

#define BINARY_THRESHOLD 180
#define VERTICAL_SKY_LINE 120
#define LANE_WIDTH 120

#define STATIC_WIDTH 320
#define STATIC_HEIGHT 240

#define MIN_VELOCITY 30
#define MAX_VELOCITY 60


#endif