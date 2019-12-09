#ifndef _ABSTRACT_H_
#define _ABSTRACT_H_

#include "core.h"

namespace tpv {
namespace abstract {

class LaneDetector{
public:
    virtual int width() = 0;
    virtual int height() = 0;

    // return the current configurations of this class
    virtual std::vector<int> get_configurations() = 0;
    virtual float get_err_angle() = 0;
    virtual void update(const TPV_CV_MAT& src, TPV_CV_MAT& dst) = 0;
    virtual void pre_process(const TPV_CV_MAT& src, TPV_CV_MAT& dst) = 0; // implements binary image inside this class
    virtual void grp_line(const VECTOR<cv::Vec4i>& lines) = 0;
    virtual void fit_lane_2_line(const TPV_CV_MAT& src, VECTOR<cv::Vec4i>& vec, float weight) = 0;

    virtual void create_track_bars();
};

class CarController {
public:
    void drive(float err);
    void stop();
};

};
};

#endif