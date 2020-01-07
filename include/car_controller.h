#ifndef _CAR_CONTROLLER_H_
#define _CAR_CONTROLLER_H_

#include "core.h"
#include "abstract.h"

namespace tpv
{
class CarControllerObject : public abstract::CarController {
    private:
        ros::NodeHandle _steer_node_handler;
        ros::NodeHandle _speed_node_handler;

        ros::Publisher _steer_pub;
        ros::Publisher _speed_pub;

#if DYNAMIC_VELOCITY
        float _max_V;
        float _min_V;
#else
        int _V;
#endif

        bool _started;

    public:
        CarControllerObject();
#if DYNAMIC_VELOCITY
        CarControllerObject(int min_v, int max_v);
        CarControllerObject(int min_v, int max_v, const std::string& steer_api, const std::string& speed_api);
#else
        CarControllerObject(int velocity);
        CarControllerObject(int velocity, const std::string& steer_api, const std::string& speed_api);
#endif

        void stop();
        void drive(float err);

#ifdef VELOCITY_TRACKBAR
        void create_v_trackbar();
#endif
};
}

#endif
