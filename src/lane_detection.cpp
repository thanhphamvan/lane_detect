#include "core.h"
#include "lane_detection.h"

namespace tpv
{
LaneDetectorObject::~LaneDetectorObject() { }

LaneDetectorObject::LaneDetectorObject()
{
    int min_thres[3] = MIN_THRESHOLD;
    int max_thres[3] = MAX_THRESHOLD;

    std::memcpy(&_min_threshold[0], &min_thres, sizeof(int) * 3);
    std::memcpy(&_max_threshold[0], &max_thres, sizeof(int) * 3);

    _binary_threshold = BINARY_THRESHOLD;
    _sky_line = VERTICAL_SKY_LINE;
    _lane_width = LANE_WIDTH;

    _width = STATIC_WIDTH;
    _height = STATIC_HEIGHT;
}

LaneDetectorObject::LaneDetectorObject(const int* min_thres,
    const int* max_thres,
    const int bin_thres,
    const int sky_line,
    const int lane_width,
    const int width,
    const int height)
{
    std::memcpy(&_min_threshold[0], min_thres, sizeof(int) * 3);
    std::memcpy(&_max_threshold[0], max_thres, sizeof(int) * 3);

    _binary_threshold = bin_thres;
    _sky_line = sky_line;
    _lane_width = lane_width;

    _width = width;
    _height = height;
}

int LaneDetectorObject::width() {
    return _width;
}

int LaneDetectorObject::height() {
    return _height;
}

void LaneDetectorObject::pre_process(const TPV_CV_MAT& src, TPV_CV_MAT& dst) {
    TPV_CV_MAT im_blurred, im_gray, im_hsv, im_thresholded, im_canny;

    static cv::Scalar lower(_min_threshold[0], _min_threshold[1], _min_threshold[2]);
    static cv::Scalar upper(_max_threshold[0], _max_threshold[1], _max_threshold[2]);

    cv::medianBlur(src, im_blurred, BLUR_KER_SIZE);
    cv::cvtColor(src, im_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(src, im_hsv, cv::COLOR_BGR2HSV);

    cv::inRange(im_hsv, lower, upper, im_thresholded);

    cv::dilate(im_thresholded, im_thresholded,
                cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(MORPH_ELLIPSE_KER_SIZE)));
    
    cv::Canny(im_gray, im_canny, CANNY_EDGES, CANNY_THRESHOLD);

    im_canny.copyTo(dst, im_thresholded);
}

}