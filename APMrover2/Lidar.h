#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "defines.h"

class Lidar {
public:
    // Constructor
    Lidar();

    bool update(mavlink_angular_distance_sensor_t &m);
    bool active();
    float check_speed(float s);
    float calc_steering(float v, int round = 0);

protected:
    class Parameters &g;
    class ParametersG2 &g2;

private:
    float check_bounds(float v, float max);

    float suggested_target_speed;
    uint32_t last_update;
    uint16_t start_angle;      // Angle in centi-degs
    uint16_t end_angle;        // Angle in centi-degs
    uint16_t angle_increment;  // Angle in centi-degs
    uint16_t min_range;
    uint16_t max_range;
    uint16_t ranges[36];
};
