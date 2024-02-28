#ifndef IMU_DRIVER
#define IMU_DRIVER

#include <iostream>
#include <stdio.h>
#include <string>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0

typedef struct {
    float roll;
    float pitch;
    float yaw;
} IMUdata;

class IMUdriver {
    private:
        const int addr;

    public:
        bool sensor_valid;

        IMUdriver();

        void read_data(IMUdata &container);

        void clear_container(IMUdata &container) {
            container.pitch = 0;
            container.roll = 0;
            container.yaw = 0;
        }
};

#endif