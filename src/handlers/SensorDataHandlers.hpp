#ifndef SENSOR_DATA_HANDLERS
#define SENSOR_DATA_HANDLERS

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "drivers/BAROdriver/bmp_388.h"
#include "drivers/IMUdriver/IMUdriver.hpp"
#include "drivers/GPSdriver/GPSdriver.hpp"
#include "drivers/RXTXdriver/RXTXdriver.hpp"

typedef struct SensorDataHandler_Def {
    int task_id;
    SemaphoreHandle_t *GENERAL_SEM;
    BMP388 *altSensor;
    IMUdriver *imu_sensor;
    IMUdata *imu_container;
    GPSdriver *gps_sensor;
    GPSdata *gps_container;
    RXTXdriver *rxtx_driver;
    RXTXdata *rxtx_container;
} SensorDataHandler;

class SensorDataHandlers {
    public:
        static void sensor_read(void *params) {
            SensorDataHandler sdh = *((SensorDataHandler *) params);
        
            while(true) {
                if(sdh.GENERAL_SEM && xSemaphoreTakeFromISR(*sdh.GENERAL_SEM, nullptr) == pdTRUE) {
                    if(sdh.task_id == 4 && sdh.rxtx_container->ch5 >= 1700) {
                        try {
                            sdh.altSensor->perform_reading();
                            // printf("task %d running on core %d.\nin alt: %lf m.\n", sdh.task_id , portGET_CORE_ID(), sdh.altSensor->get_altitude());
                            // fflush(stdout);
                        } catch(...) {
                            sdh.altSensor->sensor_valid = false;
                            xSemaphoreGiveFromISR(*sdh.GENERAL_SEM, nullptr);
                            std::cout << "Baro task has been crashed -> suspend baro task now!" << std::endl;
                            vTaskSuspend(NULL);
                        }
                    } else if(sdh.task_id == 3 && sdh.rxtx_container->ch5 >= 1700) {
                        try {
                            sdh.imu_sensor->read_data(*(sdh.imu_container));
                            // printf("task %d running on core %d.\nroll: %f, pitch: %f, yaw: %f\n", sdh.task_id , portGET_CORE_ID(), (sdh.imu_container)->roll, (sdh.imu_container)->pitch, (sdh.imu_container)->yaw);
                            // fflush(stdout);
                        } catch(...) {
                            sdh.imu_sensor->sensor_valid = false;
                            xSemaphoreGiveFromISR(*sdh.GENERAL_SEM, nullptr);
                            std::cout << "IMU task has been crashed -> suspend IMU task now!" << std::endl;
                            vTaskSuspend(NULL);
                        }
                    } else if(sdh.task_id == 1 && sdh.rxtx_container->ch5 >= 1700) {
                        try {
                            sdh.gps_sensor->read_data(*sdh.gps_container);
                            // printf("task %d running on core %d.\nN: %f, E: %f\n", sdh.task_id , portGET_CORE_ID(), (sdh.gps_container)->N, (sdh.gps_container)->E);
                            // fflush(stdout);
                        } catch(...) {
                            sdh.gps_sensor->sensor_valid = false;
                            xSemaphoreGiveFromISR(*sdh.GENERAL_SEM, nullptr);
                            std::cout << "GPS task has been crashed -> suspend GPS task now!" << std::endl;
                            vTaskSuspend(NULL);
                        }
                    } else if(sdh.task_id == 2) {
                        try {
                            sdh.rxtx_driver->read_data(*sdh.rxtx_container);
                            // printf("task %d running on core %d.\nSWC - %d\n", sdh.task_id , portGET_CORE_ID(), sdh.rxtx_container->ch5);
                            // fflush(stdout);
                        } catch(...) {
                            sdh.rxtx_driver->sensor_valid = false;
                            xSemaphoreGiveFromISR(*sdh.GENERAL_SEM, nullptr);
                            std::cout << "RXTX task has been crashed -> suspend RXTX task now!" << std::endl;
                            vTaskSuspend(NULL);
                        }
                    }
                    
                    xSemaphoreGiveFromISR(*sdh.GENERAL_SEM, nullptr);

                    if(sdh.task_id == 4) {
                        vTaskDelay(100 / portTICK_PERIOD_MS);
                    } else if(sdh.task_id == 3) {
                        vTaskDelay(5 / portTICK_PERIOD_MS);
                    } else if(sdh.task_id == 1) {
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                    } else if(sdh.task_id == 2) {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }
                } else {
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                }
            }
        }
};

#endif