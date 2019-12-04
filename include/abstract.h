#ifndef _ABSTRACT_H_
#define _ABSTRACT_H_

#include "core.h"

namespace tpv {
namespace abstract {

class LaneDetector{
public:
    virtual cv::Mat update(cv::Mat & scrMat);
    virtual float getErrAngle();
    virtual int width();
    virtual int height();

    // return the current configurations of this class
    virtual std::vector<int> get_configurations();
};

class CarController {
public:
    void drive(float err);
    void stop();
};

};
};

#endif