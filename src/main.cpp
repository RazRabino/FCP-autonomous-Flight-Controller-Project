#include <string>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "drivers/SERVOdriver/SERVOdriver.hpp"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "handlers/SensorDataHandlers.hpp"
#include "AutonomousPilot/MissionPlanner.hpp"
#include "AutonomousPilot/AutoPilot.hpp"


/*
    Defines:
*/
#define BMP388_ADDRESS 		0x77
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

/*
 * Main Task
 */
void mainTask(void *params){
    SemaphoreHandle_t GENERAL_SEM = *(SemaphoreHandle_t *) params;
    vTaskDelay(5000);

    // change this line and replace "*" by your data according to header - you can add more values if needed.
    std::string MissionRoute = "N, E, ALT, SPEED\n*, *, *, *\n*, *, *, *\n*, *, *, *\n*, *, *, *\n*, *, *, *";
    
    MissionPlanner MP(MissionRoute);
    AutoPilot AP;

    SERVOdriver roll_servo;
    const uint roll_servo_pin = 7;
    roll_servo.pwmservo_init(roll_servo_pin);

    SERVOdriver pitch_servo;
    const uint pitch_servo_pin = 9;
    pitch_servo.pwmservo_init(pitch_servo_pin);

    SERVOdriver yaw_servo;
    const uint yaw_servo_pin = 22;
    yaw_servo.pwmservo_init(yaw_servo_pin);

    SERVOdriver main_motor;
    const uint main_motor_pin = 8;
    main_motor.pwmservo_init(main_motor_pin);
    
    // // change this line and replace "0" by your data according to sea level atmospheric pressure in your area before flight.
    const int SEA_LVL_PRES = 0;
    
    BMP388 altSensor = BMP388(BMP388_ADDRESS, SEA_LVL_PRES);
    IMUdriver imu_sensor;
    IMUdata imu_container;
    imu_sensor.IMUdriver::clear_container(imu_container);
    GPSdriver gps_sensor;
    GPSdata gps_container;
    RXTXdriver com;
    RXTXdata comData;

	SensorDataHandler sdh_baro;
    sdh_baro.task_id = 4;
    sdh_baro.GENERAL_SEM = &GENERAL_SEM;
    sdh_baro.altSensor = &altSensor;
    sdh_baro.rxtx_container = &comData;

    SensorDataHandler sdh_gps;
    sdh_gps.task_id = 1;
    sdh_gps.GENERAL_SEM = &GENERAL_SEM;
    sdh_gps.gps_sensor = &gps_sensor;
    sdh_gps.gps_container = &gps_container;
    sdh_gps.rxtx_container = &comData;

    SensorDataHandler sdh_imu;
    sdh_imu.task_id = 3;
    sdh_imu.GENERAL_SEM = &GENERAL_SEM;
    sdh_imu.imu_sensor = &imu_sensor;
    sdh_imu.imu_container = &imu_container;
    sdh_imu.rxtx_container = &comData;

    SensorDataHandler sdh_rxtx;
    sdh_rxtx.task_id = 2;
    sdh_rxtx.GENERAL_SEM = &GENERAL_SEM;
    sdh_rxtx.rxtx_driver = &com;
    sdh_rxtx.rxtx_container = &comData;

    SensorDataHandlers sdh;

    vTaskDelay(15000);
	printf("Main task started\n");
    fflush(stdout);

	//Bind to CORE 1
	TaskHandle_t GPStask;
	UBaseType_t coreMask = 0x2;
    xTaskCreate(sdh.sensor_read, "GPSdriver task", 256, &sdh_gps, TASK_PRIORITY, &(GPStask));
    vTaskCoreAffinitySet(GPStask, coreMask);

    printf("GPSdriver task created\n");
    fflush(stdout);

	TaskHandle_t IMUtask;
    xTaskCreate(sdh.sensor_read, "IMUdriver task", 256, &sdh_imu, TASK_PRIORITY, &(IMUtask));
    vTaskCoreAffinitySet(IMUtask, coreMask);

    printf("IMUdriver task created\n");
    fflush(stdout);

	TaskHandle_t BAROtask;
    xTaskCreate(sdh.sensor_read, "BAROdriver task", 256, &sdh_baro, TASK_PRIORITY, &(BAROtask));
    vTaskCoreAffinitySet(BAROtask, coreMask);

    printf("BAROdriver task created\n");
    fflush(stdout);
    
    TaskHandle_t RXTXtask;
    xTaskCreate(sdh.sensor_read, "RXTXdriver task", 256, &sdh_rxtx, TASK_PRIORITY, &(RXTXtask));
    vTaskCoreAffinitySet(RXTXtask, coreMask);

    printf("RXTXdriver task created\n");
    fflush(stdout);

    bool planeLanded = false;
    bool sensorsValid = true;

    vTaskDelay(10000);
    while(!sdh_baro.altSensor->sensor_valid || !sdh_gps.gps_sensor->sensor_valid || !sdh_imu.imu_sensor->sensor_valid || !sdh_rxtx.rxtx_driver->sensor_valid) {
        vTaskDelay(150);
    }

    printf("Starting flight!\n");
    fflush(stdout);

    xSemaphoreGiveFromISR(GENERAL_SEM, nullptr);
	while (true) {
        try {
            if(GENERAL_SEM && xSemaphoreTakeFromISR(GENERAL_SEM, nullptr) == pdTRUE) {
                if(!sdh_baro.altSensor->sensor_valid || !sdh_gps.gps_sensor->sensor_valid || !sdh_imu.imu_sensor->sensor_valid) sensorsValid = false;

                if(sdh_gps.gps_container->fixState == 1 && !planeLanded && comData.ch5 >= 1700 && sensorsValid) {

                    WayPointData tempWP{gps_container.N, gps_container.E, sdh_baro.altSensor->altitude, gps_container.speed};
                    if(!MP.wpsEqual(tempWP, WayPointData{-1.0, -1.0, -1.0, -1.0}) && MP.isInWPZone(tempWP)) {
                        MP.setNextWP();
                    }
                    
                    if(!MP.wpsEqual(tempWP, WayPointData{-1.0, -1.0, -1.0, -1.0})) {
                        uint roll_angle = AP.compute_roll(sdh_imu.imu_container->roll);
                        uint pitch_angle = AP.compute_pitch(MP, sdh_imu.imu_container->pitch, sdh_baro.altSensor->altitude, gps_container);
                        uint yaw_angle = AP.compute_yaw(MP, imu_container, gps_container);
                        uint throttle_pwm = AP.compute_throttle(MP, sdh_baro.altSensor->altitude, gps_container);

                        roll_servo.pwmservo_write_micros(roll_servo_pin, (roll_angle));
                        pitch_servo.pwmservo_write_micros(pitch_servo_pin, (pitch_angle));
                        yaw_servo.pwmservo_write_micros(yaw_servo_pin, (yaw_angle));
                        main_motor.pwmservo_write_micros(main_motor_pin, (throttle_pwm));
                    } else {
                        planeLanded = true;
                    }
                } else if(comData.ch5 >= 1300 || !sensorsValid) {
                    if(comData.ch1 >= 1000 && comData.ch1 <= 2000) roll_servo.pwmservo_write_micros(roll_servo_pin, (comData.ch1));
                    if(comData.ch2 >= 1000 && comData.ch2 <= 2000) pitch_servo.pwmservo_write_micros(pitch_servo_pin, (comData.ch2));
                    if(comData.ch4 >= 1000 && comData.ch4 <= 2000) yaw_servo.pwmservo_write_micros(yaw_servo_pin, (comData.ch4));
                    if(comData.ch3 >= 1000 && comData.ch3 <= 2000) main_motor.pwmservo_write_micros(main_motor_pin, (comData.ch3));
                } else {
                    roll_servo.pwmservo_write_micros(roll_servo_pin, (1500));
                    pitch_servo.pwmservo_write_micros(pitch_servo_pin, (1500));
                    yaw_servo.pwmservo_write_micros(yaw_servo_pin, (1500));
                    main_motor.pwmservo_write_micros(main_motor_pin, (1000));
                }
                if(GENERAL_SEM) xSemaphoreGiveFromISR(GENERAL_SEM, nullptr);
                vTaskDelay(10);
            }
        } catch(...) {
            printf("emergancy Mode!\n");
            fflush(stdout);

            while(true) {
                if(comData.ch1 >= 1000 && comData.ch1 <= 2000) roll_servo.pwmservo_write_micros(roll_servo_pin, (comData.ch1));
                if(comData.ch2 >= 1000 && comData.ch2 <= 2000) pitch_servo.pwmservo_write_micros(pitch_servo_pin, (comData.ch2));
                if(comData.ch4 >= 1000 && comData.ch4 <= 2000) yaw_servo.pwmservo_write_micros(yaw_servo_pin, (comData.ch4));
                if(comData.ch3 >= 1000 && comData.ch3 <= 2000) main_motor.pwmservo_write_micros(main_motor_pin, (comData.ch3));
            }
        }
	}
}

/***
 * Launch the tasks and scheduler
 */
void vLaunch(void *params) {
	// Init Main task and Data Container
    TaskHandle_t task;

    std::cout << "Starting flight controller init proccess" << std::endl;
    xTaskCreate(mainTask, "MainThread", 500, params, TASK_PRIORITY, &task);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

/***
 * Main
 * @return
 */
int main( void )
{
	// Setup serial over USB and give a few seconds to settle before we start
    stdio_init_all();
    sleep_ms(3000);
    std::cout << "Board ready, Starting FreeRtos Lib" << std::endl;

    //Start tasks and scheduler
    std::string rtos_name;
#if ( portSUPPORT_SMP == 1 )
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if ( portSUPPORT_SMP == 1 ) && ( configNUM_CORES == 2 )
    std::cout << rtos_name << " on both cores" << std::endl;
#else
    std::cout << "Starting " << rtos_name << " on core 0" << std::endl;
#endif

	i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    std::cout << std::endl;

    SemaphoreHandle_t GENERAL_SEM = xSemaphoreCreateBinary();

    vLaunch(&GENERAL_SEM);

    return 0;
}