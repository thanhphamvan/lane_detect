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

        float _max_V;
        float _min_V;

        bool _started;

    public:
        CarControllerObject();
        CarControllerObject(int min_v, int max_v);
        CarControllerObject(int min_v, int max_v, const std::string& steer_api, const std::string& speed_api);

        void stop();
        void drive(float err);
};
}

#endif
