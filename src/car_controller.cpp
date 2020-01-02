#include "car_controller.h"
#include "utils.h"

namespace tpv {

CarControllerObject::CarControllerObject() {
    _max_V = MAX_VELOCITY;
    _min_V = MIN_VELOCITY;

    _steer_pub = _steer_node_handler.advertise<FLOAT_MSG_TYPE>(SET_STEER_API_HOOK,
                                                                DEFAULT_QUEUE_SIZE);

    _speed_pub = _speed_node_handler.advertise<FLOAT_MSG_TYPE>(SET_SPEED_API_HOOK,
                                                                DEFAULT_QUEUE_SIZE);
}

CarControllerObject::CarControllerObject(int min_v, int max_v) {
    _min_V = min_v;
    _max_V = max_v;

    _steer_pub = _steer_node_handler.advertise<FLOAT_MSG_TYPE>(SET_STEER_API_HOOK,
                                                                DEFAULT_QUEUE_SIZE);

    _speed_pub = _speed_node_handler.advertise<FLOAT_MSG_TYPE>(SET_SPEED_API_HOOK,
                                                                DEFAULT_QUEUE_SIZE);
}

CarControllerObject::CarControllerObject(int min_v, int max_v,
                                         const std::string& steer_api,
                                         const std::string& speed_api) {
    _min_V = min_v;
    _max_V = max_v;

    _steer_pub = _steer_node_handler.advertise<FLOAT_MSG_TYPE>(steer_api,
                                                                DEFAULT_QUEUE_SIZE);

    _speed_pub = _speed_node_handler.advertise<FLOAT_MSG_TYPE>(speed_api,
                                                                DEFAULT_QUEUE_SIZE);
}

void CarControllerObject::stop() {
    if (_started)
    {
        FLOAT_MSG_TYPE spd;
        spd.data = -1;
        _speed_pub.publish(spd);
    }
}

void CarControllerObject::drive(float err) {
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