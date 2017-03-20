#include "AP_Wheelspeed.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define ARDUINO_I2C_ADDR 0x46

// table of user settable parameters
const AP_Param::GroupInfo AP_Wheelspeed::var_info[] = {
    // @Param: USE
    // @DisplayName: Wheelspeed use
    // @Description: use wheel speed for control
    // @Values: 1:Use,0:Don't Use
    // @User: Standard
    AP_GROUPINFO("USE", 0, AP_Wheelspeed, _use, 0),

    AP_GROUPEND
};

AP_Wheelspeed::AP_Wheelspeed() {
    AP_Param::setup_object_defaults(this, var_info);
}

// probe and initialise the sensor
void AP_Wheelspeed::init() {
    const struct {
        uint8_t bus;
        uint8_t addr;
    } addresses[] = {
        { 1, ARDUINO_I2C_ADDR },
        { 0, ARDUINO_I2C_ADDR },
    };
    bool found = false;

    for (uint8_t i = 0; i < ARRAY_SIZE(addresses); i++) {
        _dev = hal.i2c_mgr->get_device(addresses[i].bus, addresses[i].addr);
        if (!_dev) {
            continue;
        }
        if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            continue;
        }

        // lots of retries during probe
        _dev->set_retries(10);

        hal.scheduler->delay(10);
        _read();

        _dev->get_semaphore()->give();

        found = (_last_sample_time_ms != 0);
        if (found) {
            printf("Wheelspeed: Found Arduino on bus %u address 0x%02x\n", addresses[i].bus, addresses[i].addr);
            break;
        }
    }

    if (!found) {
        printf("Wheelspeed: no sensor found\n");
        _enabled = 0;
        return;
    }

    // drop to 2 retries for runtime
    _dev->set_retries(2);

    _dev->register_periodic_callback(100000,
        FUNCTOR_BIND_MEMBER(&AP_Wheelspeed::_read, void));
    _enabled = 0;
    return;
}

// read the values from the sensor
// 10Hz timer
void AP_Wheelspeed::_read() {
    uint8_t data[2];

    if (!_dev->transfer(nullptr, 0, data, sizeof(data))) {
        return;
    }

    int16_t i16 = data[1] << 8 | data[0];

    _speed = (float)i16 / 100.0;

    _last_sample_time_ms = AP_HAL::millis();
}

// return the current wheel speed
float AP_Wheelspeed::get_wheel_speed() {
    if (!enabled()) {
        return 0.0;
    }

    if ((AP_HAL::millis() - _last_sample_time_ms) > 500) {
        return 0.0;
    }

    return _speed;
}
