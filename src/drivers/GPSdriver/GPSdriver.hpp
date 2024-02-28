#ifndef GPS_DRIVER
#define GPS_DRIVER

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include "pico/stdlib.h"


typedef struct {
    // gps data time stamp
    int hr;
    int min;
    float sec;

    // gps coordinates
    double N;
    double E;

    // gps info
    int fixState;

    // calculated data
    float speed; // units: m/s
} GPSdata;

class GPSdriver {
    private:
        const double a = 6378137.0;   // Semi-major axis
        const double b = 6356752.314; // Semi-minor axis
        const double f = 1 / 298.257223563; // Flattening
        const double KnotsToMeterPerSec = 0.514444;

        double degreesToRadians(double degrees) {
            return degrees * 3.14159265358979323846 / 180.0;
        }

        enum type_len {
            message_len = 256,
            configuration_len = 46
        };

        // Define uart properties for the GPS
        const uint32_t GPS_BAUDRATE = 9600; 
        const uint8_t GPS_TX = 0, GPS_RX = 1;
        const uint8_t DATABITS = 8;
        const uint8_t STARTBITS = 1;
        const uint8_t STOPBITS = 1;

        // Calculate the delay between bytes
        const uint8_t BITS_PER_BYTE = STARTBITS + DATABITS + STOPBITS;
        const uint32_t MICROSECONDS_PER_SECOND = 1000000;
        const uint32_t GPS_DELAY = BITS_PER_BYTE * MICROSECONDS_PER_SECOND / GPS_BAUDRATE;

        char message[message_len];
        const char CONFIGURATIONS[configuration_len];
        uart_inst_t *gps_uart;

    public:
        bool sensor_valid;

        GPSdriver();

        size_t uart_read_line(uart_inst_t *uart, char *buffer, const size_t max_length);

        void send_with_checksum(uart_inst_t *uart, const char *message, const size_t length);

        void read_data(GPSdata &container);

        void clear_container(GPSdata &container) {
            container.hr = -1; container.min = -1; container.sec = -1;
            container.N = -1; container.E = -1; container.fixState = -1;
        }
};

#endif