#include "core.h"
#include "abstract.h"

#define SQUARE(x)(x*x)

float inline euclide_dist(cv::Vec4i l)
{
    return sqrt(SQUARE(l[0] - l[2]) + SQUARE(l[1] - l[3]));
}

float inline angle(cv::Vec4i l) {
    return (atan2(l[2] - l[0], l[3] - l[1]) * 180) / CV_PI;
}

namespace tpv {

class CarControllerImpl : public abstract::CarController {
    private:
        ros::NodeHandle _steer_node_handler;
        ros::NodeHandle _speed_node_handler;

        ros::Publisher _steer_pub;
        ros::Publisher _speed_pub;

        float _max_V;
        float _min_V;

        bool _started;

    public:
        CarControllerImpl() 
        {
            _max_V = MAX_VELOCITY;
            _min_V = MIN_VELOCITY;

            _steer_pub = _steer_node_handler.advertise<FLOAT_MSG_TYPE>(SET_STEER_API_HOOK,
                                                                       DEFAULT_QUEUE_SIZE);

            _speed_pub = _speed_node_handler.advertise<FLOAT_MSG_TYPE>(SET_SPEED_API_HOOK,
                                                                       DEFAULT_QUEUE_SIZE);
        };

        CarControllerImpl(int min_v, int max_v) {
            _min_V = min_v;
            _max_V = max_v;

            _steer_pub = _steer_node_handler.advertise<FLOAT_MSG_TYPE>(SET_STEER_API_HOOK,
                                                                       DEFAULT_QUEUE_SIZE);

            _speed_pub = _speed_node_handler.advertise<FLOAT_MSG_TYPE>(SET_SPEED_API_HOOK,
                                                                       DEFAULT_QUEUE_SIZE);
        }

        ~CarControllerImpl() {}

        virtual void stop()
        {
            if (_started)
            {
                FLOAT_MSG_TYPE spd;
                spd.data = -1;
                _speed_pub.publish(spd);
            }
        }

        virtual void drive(float err) 
        {
            float v = _max_V;
            _started = true;

            if (abs(err) > ERR_THRESHOLD)
            {
                v -= abs(err);
            }

            if (v < _min_V)
            {
                v = _min_V;
            }

            #ifdef DEBUG_CONST_V
            v = 15;
            #endif

            FLOAT_MSG_TYPE angle;
            FLOAT_MSG_TYPE speed;

            angle.data = - err;
            speed.data = v;

            _steer_pub.publish(angle);
            _speed_pub.publish(speed);
        }


};

class LaneDetectorImpl : public abstract::LaneDetector {
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

    public:
        ~LaneDetectorImpl() {   }

        // default constructor
        LaneDetectorImpl()
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

        // with proper signature
        LaneDetectorImpl(const int* min_thres,
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

        virtual int width() {
            return _width;
        }

        virtual int height() {
            return _height;
        }
};
};