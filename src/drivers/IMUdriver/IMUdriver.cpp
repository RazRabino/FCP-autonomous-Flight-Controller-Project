#include "IMUdriver.hpp"


IMUdriver::IMUdriver() : addr(0x28) {
    // Check to see if connection is correct
    sleep_ms(1000); // Add a short delay to help BNO005 boot up
    uint8_t reg = 0x00;
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, chipID, 1, false);

    if(chipID[0] != 0xA0){
        while(1){
            printf("Chip ID Not Correct - Check Connection!");
            sleep_ms(5000);
        }
    }

    // Reset all interrupt status bits and use internal oscillator
    uint8_t data[2];
    data[0] = 0x3F;
    data[1] = 0b1000000;
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);

    // Configure Power Mode - Normal
    data[0] = 0x3E;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);
    sleep_ms(50);

    // Defaul Axis Configuration
    data[0] = 0x41;
    data[1] = 0x24;
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);

    // Default Axis Signs
    data[0] = 0x42;
    data[1] = 0x06;
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);

    // Set units to m/s^2
    data[0] = 0x3B;
    data[1] = 0b0001000;
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);
    sleep_ms(30);

    // Set BNO055 to config mode
    data[0] = 0x3D;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);
    sleep_ms(100);

    // Set operation to NDOF
    data[0] = 0x3D;
    data[1] = 0x0C;
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);
    sleep_ms(100);

    sensor_valid = true;
}

void IMUdriver::read_data(IMUdata &container) {
    uint8_t euler_data[6];
    uint8_t euler_register = 0x1A;

    int wr = i2c_write_blocking(I2C_PORT, addr, &euler_register, 1, false);
    int rr = i2c_read_blocking(I2C_PORT, addr, euler_data, 6, true);

    if(wr != PICO_ERROR_GENERIC && rr != PICO_ERROR_GENERIC) {
        int16_t yaw_angle = (euler_data[1] << 8) | euler_data[0];
        int16_t pitch_angle = (euler_data[3] << 8) | euler_data[2];
        int16_t roll_angle = (euler_data[5] << 8) | euler_data[4];
        float scale = 1.0 / 16.0;
        float yaw = yaw_angle * scale;
        float pitch = pitch_angle * scale;
        float roll = roll_angle * scale;

        container.roll = roll; container.pitch = pitch; container.yaw = yaw;
    }
}

