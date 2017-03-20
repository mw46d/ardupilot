#include <MsTimer2.h>
#include <Wire.h>

#define ENC_PIN_A PD2  //pin 2
#define ENC_PIN_B PD3  //pin 3

#define WHEEL_DIA 0.2            // diameter in meters
#define COUNTS_PER_REV 80        // counts per revolution
#define TIME  100               // time for one collection in ms
#define PI  3.14159265359

// 2m/s / (WHEEL_DIA * PI) * COUNTS_PER_REV -> 254.647908947 enc/s -> 25.4647908947 enc/(0.1 s)

#define SPEED_PER_ENC 1000.0 / TIME / COUNTS_PER_REV * PI * WHEEL_DIA

volatile long enc_pos = 0L;
volatile float speed;            // in m/s
volatile int speed_int;          // in cm/s

static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table

ISR (PCINT2_vect) {
    static uint8_t enc_last = 0;

    enc_last <<= 2; //shift previous state two places
    enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

    enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

void initEncoder() {
    // set as inputs
    DDRD &= ~(1 << ENC_PIN_A);
    DDRD &= ~(1 << ENC_PIN_B);

    // enable pull up resistors
    PORTD |= (1 << ENC_PIN_A);
    PORTD |= (1 << ENC_PIN_B);

    // tell pin change mask to listen to encoder pins
    PCMSK2 |= (1 << ENC_PIN_A) | (1 << ENC_PIN_B);

    // enable PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE2);
}

/* Wrap the encoder reading function */
long readEncoder() {
    long l = enc_pos;

    enc_pos = 0L;

    return l;
}

void calc_speed() {
    static float av_enc = 0.0;
    long l = readEncoder();

    if (l == 0) {
        av_enc = 0.0;
        speed = 0.0;
    }
    else {
        av_enc = (4.0 * av_enc + (float)l) / 5.0;
        speed = av_enc * SPEED_PER_ENC;
    }

    speed_int = (int)(speed * 100.0);
}

void i2c_request() {
  byte buf[2];

  buf[0] = (byte)(speed_int & 0xff);
  buf[1] = (byte)(speed_int >> 8);
  
  Wire.write(buf, 2);
}

void setup() {
    // Serial.begin(115200);

    initEncoder();

    Wire.begin(i0x46);
    Wire.onRequest(i2c_request);
    
    MsTimer2::set(TIME, calc_speed);
    MsTimer2::start();
}

void loop() {
    delay(50000);
}
