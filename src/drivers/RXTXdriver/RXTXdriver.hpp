#ifndef RXTX_DRIVER
#define RXTX_DRIVER

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

typedef struct {
    uint ch1;
    uint ch2; 
    uint ch3; 
    uint ch4; 
    uint ch5; 
    uint ch6;
} RXTXdata;

class RXTXdriver {
    private:
        const uint PWM_PINS[6] = {13, 15, 27, 21, 19, 17}; // channel 1 = gpio13, channel 6 = gpio17

    public:
        bool sensor_valid = true;
        
        RXTXdriver(){}

        void measure_duty_cycle(float measured_duty_cycle[]);

        void read_data(RXTXdata &container);
};

#endif