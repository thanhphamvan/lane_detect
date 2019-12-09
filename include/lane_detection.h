#ifndef _LANE_DETECTION_H_
#define _LANE_DETECTION_H_

#include <string>

#include "core.h"
#include "abstract.h"

namespace tpv
{
class LaneDetectorObject : public abstract::LaneDetector
{
    private:
        // internal configurations
        int _min_threshold[3];
        int _max_threshold[3];
        int _binary_threshold;
        int _sky_line;
        int _lane_width;

        int _width;
        int _height;

        // internal memories
        cv::Vec4i _left_lane;
        cv::Vec4i _right_lane;
        cv::Vec4i _pre_lane;

        cv::Point _car_position;

        float _err_angle(cv::Point);

    public:
        ~LaneDetectorObject();
        LaneDetectorObject();
        LaneDetectorObject(const int* min_thres,
                    const int* max_thres,
                    const int bin_thres,
                    const int sky_line,
                    const int lane_width,
                    const int width,
                    const int height);
        LaneDetectorObject(const std::string& json_config_path);

        int width();
        int height();

        std::vector<int> get_configurations();

        float get_err_angle();
        void update(const TPV_CV_MAT& src, TPV_CV_MAT& dst);
        void pre_process(const TPV_CV_MAT& src, TPV_CV_MAT& dst);
        void grp_line(const VECTOR<cv::Vec4i>& lines);
        void fit_lane_2_line(const TPV_CV_MAT& src, VECTOR<cv::Vec4i>& vec, float weight);

        void create_track_bars();
};  
}

#endif
