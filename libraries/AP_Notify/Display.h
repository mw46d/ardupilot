#pragma once

#include "NotifyDevice.h"

#define ROW(Y)    ((Y * 10) + 6)
#define COLUMN(X) ((X *  7) + 0)

#define DISPLAY_MESSAGE_SIZE 19

class Display_Backend;

class Display: public NotifyDevice {
public:
    friend class Display_Backend;

    bool init(void);
    void update();

private:
    void draw_char(uint16_t x, uint16_t y, const char c);
    void draw_text(uint16_t x, uint16_t y, const char *c);
    bool update_all();
    bool update_arm(uint8_t r, uint8_t c = 0);
    bool update_prearm(uint8_t r, uint8_t c = 0);
    bool update_gps(uint8_t r, uint8_t c = 0);
    bool update_gps_sats(uint8_t r, uint8_t c = 0);
    bool update_ekf(uint8_t r, uint8_t c = 0);
    bool update_battery(uint8_t r, uint8_t c = 0);
    bool update_mode(uint8_t r, uint8_t c = 0);
    bool update_text(uint8_t r, uint8_t c = 0);
    bool update_text_empty(uint8_t r, uint8_t c = 0);

    Display_Backend *_driver;

    bool _healthy;

    uint8_t _mstartpos; // ticker shift position
    uint8_t _movedelay; // ticker delay before shifting after new message displayed
    uint8_t _screenpage;

    // stop showing text in display after this many millis:
    const uint16_t _send_text_valid_millis = 20000;

    // MARCO
    double _mw_battery_voltage = 255.0;
    uint32_t _mw_flight_mode = 0;
    uint32_t _mw_text_updated = 0;
    uint8_t _mw_armed = 5;
    uint8_t _mw_gps_status = 10;
    uint8_t _mw_gps_num_sats = 0;
    uint8_t _mw_ekf_bad = 5;
};

