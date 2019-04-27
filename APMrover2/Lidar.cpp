#include "Lidar.h"
#include "Rover.h"

#define AVOID_TIME 750
#define STRAIGHT_TIME 750

Lidar::Lidar() :
    g(rover.g),
    g2(rover.g2),
    suggested_target_speed(-1.0),
    last_update(0)
{ }

bool Lidar::update(mavlink_angular_distance_sensor_t &m) {
    int failed_reads = 0;

    this->last_update = millis();
    this->start_angle = m.start_angle;
    this->end_angle = m.end_angle;
    this->angle_increment = m.angle_increment;
    this->min_range = m.min_distance;
    this->max_range = m.max_distance;

    for (int i = 0; i < 36; i++) {
        this->ranges[i] = 0;
    }

    for (int i = 0; i < 14; i++) {
        if (m.covariances[i] > 0) {
            if (m.ranges[i] < m.min_distance || m.ranges[i] > m.max_distance) {
                failed_reads++;
                this->ranges[(42 - i) % 36] = 0;
                // test
                m.ranges[i] = this->ranges[(42 - i) % 36];
                m.covariances[i] = 0;
            }
            else {
                this->ranges[(42 - i) % 36] = m.ranges[i];
                // test
                m.ranges[i] = this->ranges[(42 - i) % 36];
                m.covariances[i] = 5;
            }
        }
        else {
            this->ranges[(42 - i) % 36] = 0;
            // test
            m.ranges[i] = this->ranges[(42 - i) % 36];
            m.covariances[i] = 0;
        }
    }

    if (failed_reads > 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MW lidar update() failed_reads= %d", failed_reads);
    }

    return true;
}

bool Lidar::active() {
    uint32_t t = millis();
    static uint32_t last_log = 0;

    bool ret = ((t - this->last_update) < 250) ||
      ((t - this->detected_time_ms) < (AVOID_TIME * (this->suggested_target_speed > 0.0 ? (2.0 / this->suggested_target_speed) : 1.0))) ||
      ((t - this->straight_time_ms) < (STRAIGHT_TIME * (this->suggested_target_speed > 0.0 ? (2.0 / this->suggested_target_speed) : 1.0)));

    /*
    if (last_log + 1000 < t) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "MW lidar active(%d, %d, %d -- %f)= %d", this->last_update, this->detected_time_ms, this->straight_time_ms, this->suggested_target_speed, ret);
        last_log = t;
    }
    */

    return ret;
}

float Lidar::check_bounds(float v, float max) {
    if (v > max) {
        return max;
    }
    else if (v < - max) {
        return - max;
    }

    return v;
}

float Lidar::check_speed(float s) {
    if (this->suggested_target_speed < 0.0) {
        return s;
    }

    return this->suggested_target_speed;
}

float Lidar::calc_steering(float v, int round) {
    uint32_t t = millis();

    // v ---> pos
    // <--- v neg

    if ((t - this->detected_time_ms) < AVOID_TIME) {
        return this->avoid_angle;
    }

    if (this->detected_time_ms > 0) {
        this->straight_time_ms = t;
        this->detected_time_ms = 0;
    }

    if ((t - this->straight_time_ms) < STRAIGHT_TIME && round == 0) {
        v = 0.0;
    }

    if (round > 3) {
        // short circuit
        this->avoid_angle = v;
        this->detected_time_ms = t;
        return v;
    }

    int i;
    int max_problems;
    int16_t i_array[5];
    uint16_t r_array[5];
    int8_t order[5];

    i = (int)(v * 45.0);
    max_problems = 0;
    this->suggested_target_speed = -1.0;

    i_array[0] = (360 + 19 - i) / 10 % 36;
    i_array[1] = (360 + 9 - i) / 10 % 36;
    i_array[2] = (360 - i) / 10 % 36;
    i_array[3] = (360 - 9 - i) / 10 % 36;
    i_array[4] = (360 - 19 - i) / 10 % 36;

    r_array[0] = this->ranges[i_array[0]];
    r_array[1] = this->ranges[i_array[1]];
    r_array[2] = this->ranges[i_array[2]];
    r_array[3] = this->ranges[i_array[3]];
    r_array[4] = this->ranges[i_array[4]];

    for (i = 0; i < 5; i++) {
        if (r_array[i] > 0 && r_array[i] < 200) {
            int oi;

            for (oi = 0; oi < max_problems && r_array[order[oi]] < r_array[i]; oi++);

            if (oi < max_problems) {
                for (int oie = max_problems + 1; oie > oi; oie--) {
                    order[oie] = order[oie - 1];
                }
            }

            order[oi] = i;
            max_problems++;
        }
    }

    if (max_problems == 0) {
        // no obstacle - go ahead with the planned angle
        if (round > 0) {
            this->avoid_angle = v;
            this->detected_time_ms = t;
        }

        return v;
    }

    if (this->ranges[order[0]] < 1.5) {
        // Slow down when there are obstacles near by
        this->suggested_target_speed = 1.0;
    }

    /*
    if (this->ranges[order[0]] < 1.0) {
        // Slow down when there are obstacles near by
        this->suggested_target_speed = 0.5;
    }
    */

    // gcs().send_text(MAV_SEVERITY_CRITICAL, "MW(%d) %.3f %d %d %d %d %d --> %d", round, v, i_array[0], i_array[1], i_array[2], i_array[3], i_array[4], order[0]);
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "MW %d %d %d %d %d -> %d", r_array[0], r_array[1], r_array[2], r_array[3], r_array[4], order[0]);

    // simple minded!!
    if (v > 0) {
        if (v < 0.778) {
            switch (order[0]) {
            case 0:
            case 1:
                return this->calc_steering(v + 0.333 * g.turn_max_g, round + 1); // Try 15 deg more
                break;
            case 4:
                return this->calc_steering(v - 0.222 * g.turn_max_g, round + 1); // Try 10 deg less
                break;
            case 2:
            case 3:
                return this->calc_steering(v + 0.444 * g.turn_max_g, round + 1); // Try 20 deg more
                break;
            }
        }
        else {
            switch (order[0]) {
            case 0:
            case 1:
                if (round > 0) {
                    this->avoid_angle = v;
                    this->detected_time_ms = t;
                }
                return v; // Try the original and hope we faind a way out
                break;
            case 4:
                return this->calc_steering(v - 0.222 * g.turn_max_g, round + 1); // Try 10 deg less
                break;
            case 2:
            case 3:
                return this->calc_steering(v - 0.444 * g.turn_max_g, round + 1); // Try 20 deg less
                break;
            }
        }
    }
    else {
        if (v > -0.778) {
            switch (order[0]) {
            case 4:
            case 3:
                return this->calc_steering(v - 0.333 * g.turn_max_g, round + 1); // Try 15 deg more
                break;
            case 0:
                return this->calc_steering(v + 0.222 * g.turn_max_g, round + 1); // Try 10 deg less
                break;
            case 1:
            case 2:
                return this->calc_steering(v - 0.444 * g.turn_max_g, round + 1); // Try 20 deg more
                break;
            }
        }
        else {
            switch (order[0]) {
            case 4:
            case 3:
                if (round > 0) {
                    this->avoid_angle = v;
                    this->detected_time_ms = t;
                }
                return v; // Try the original and hope we find a way out
                break;
            case 0:
                return this->calc_steering(v + 0.222 * g.turn_max_g, round + 1); // Try 10 deg less
                break;
            case 1:
            case 2:
                return this->calc_steering(v + 0.444 * g.turn_max_g, round + 1); // Try 20 deg less
                break;
            }
        }
    }

    // Not really reached!
    return v;
}
