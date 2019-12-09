#include "lane_detection.h"

#include "utils.h"

#define SQUARE(x)(x*x)

float inline euclide_dist(cv::Vec4i l)
{
    return sqrt(SQUARE(l[0] - l[2]) + SQUARE(l[1] - l[3]));
}

float inline angle(cv::Vec4i l) {
    return (atan2(l[2] - l[0], l[3] - l[1]) * 180) / CV_PI;
}

cv::Point inline get_point(cv::Vec4i l, float y)
{
    return cv::Point((y - l[1]) * (l[0] - l[2]) / (l[1] - l[3]) + l[0], y);
}

int int_cmpr(const int a, const int b) 
{
    if (a < b)
        return -1;

    if (a > b)
        return 1;

    return 0;
}

namespace tpv
{
static cv::Vec4i NIL_LANE = cv::Vec4i();

struct DistanceCalc {
    bool operator()(cv::Vec4i a, cv::Vec4i b);
};

bool DistanceCalc::operator()(cv::Vec4i a, cv::Vec4i b)
{
    return abs(angle(a) - angle(b)) < DISTANCE_CALC_BIN_THRESHOLD;
}


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

void LaneDetectorObject::update(const TPV_CV_MAT& src, TPV_CV_MAT& dst)
{

}

float LaneDetectorObject::get_err_angle()
{
     cv::Point dst(_width / 2, _height / 2);

    cv::Point p1 = get_point(_left_lane, _height / 2);
    cv::Point p2 = get_point(_right_lane, _width / 2);

    int pr = _pre_lane != NIL_LANE ? get_point(_pre_lane, _height / 2).x : _width / 2;

    if (_left_lane != NIL_LANE && _right_lane != NIL_LANE)
    {
        if (abs((p1.x + p2.x) / 2 - pr) < 30)
        {
            dst.x = (p1.x + p2.x) / 2;
        }
        else
        {
            dst.x = ((p1.x + p2.x) / 2 + pr) / 2;
        }
    }
    else if (_right_lane != NIL_LANE)
    {
        dst.x = p2.x - _lane_width / 2;
    }
    else if (_left_lane != NIL_LANE)
    {
        dst.x = p1.x + _lane_width / 2;
    }

    return _err_angle(dst);
}

float LaneDetectorObject::_err_angle(cv::Point p)
{
    if (p.x == _car_position.x)
    {
        return 0;
    }

    if (p.y == _car_position.y) {
        return (p.x < _car_position.x ? -90 : 90);
    }

    double dx = p.x - _car_position.x;
    double dy = _car_position.y - p.y;

    if (dx < 0) {
        return - atan(-dx / dy) * 180 / CV_PI;
    } else {
        return atan(dx / dy) * 180 / CV_PI;
    }

}

void grp_line(const VECTOR<cv::Vec4i>& lines)
{
    if (lines.size() == 0)
    {
        return;
    }

    VECTOR<int> labels;

    int count = cv::partition(lines, labels, DistanceCalc());

    int count_line[count];
    int idx[count];
    cv::Vec4i mean[count];

    std::iota(idx, idx + count, 0);

    for (int i = 0; i < labels.size(); i++)
    {
        count_line[labels[i]]++;
        mean[labels[i]] = cv::Vec4i(0, 0, 0, 0);
    }

    // sorting
    c_bsort<int, int>(count_line, idx, count, int_cmpr);
}
}