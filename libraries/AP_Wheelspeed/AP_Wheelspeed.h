#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>

#include <utility>

class AP_Wheelspeed {
public:
    AP_Wheelspeed();
    ~AP_Wheelspeed(void) {}

    void init();

    float get_wheel_speed();

    bool use(void) const {
        return enabled() && _use;
    }

    bool enabled(void) const {
        return _enabled;
    }

    void disable(void) {
        _enabled = 0;
    }

    // return time in ms of last update
    uint32_t last_update_ms(void) const { return _last_sample_time_ms; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    void _read();

    float _speed;
    uint32_t _last_sample_time_ms;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    AP_Int8 _enabled;
    AP_Int8 _use;
};
