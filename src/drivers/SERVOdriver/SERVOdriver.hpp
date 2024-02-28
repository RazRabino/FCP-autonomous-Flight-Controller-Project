#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include "hardware/clocks.h"

#pragma once

#define PWMSERVO_NEUTRAL 1500
#define PWMSERVO_MIN 1000
#define PWMSERVO_MAX 2000

#define _deg_2_micros_slope (PWMSERVO_MAX - PWMSERVO_MIN)/(180)

class SERVOdriver {
    public:
        int _improved_map(int value, int minIn, int maxIn, int minOut, int maxOut);

        bool pwmservo_init(uint pin);

        void pwmservo_write_micros(uint pin, uint val) {
            pwm_set_gpio_level(pin, val);
        };

        void pwmservo_write(uint pin, uint angle) {
            pwm_set_gpio_level(pin, _clamp_val((_deg_2_micros_slope*_clamp_val(angle, -90, 90)) + PWMSERVO_NEUTRAL, 1000, 2000));
        };

        int _clamp_val(uint val, uint min, uint max) {
            const int n = val < min ? min : val;
            return n > max ? max : n;
        };
};