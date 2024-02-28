add_executable(${NAME}
    main.cpp
    handlers/SensorDataHandlers.hpp

    AutonomousPilot/MissionPlanner.cpp
    AutonomousPilot/MissionPlanner.hpp

    AutonomousPilot/AutoPilot.cpp
    AutonomousPilot/AutoPilot.hpp

    drivers/BAROdriver/bmp_388.cpp
    drivers/BAROdriver/bmp_388.h
    drivers/BAROdriver/driver/bmp3.c
    drivers/BAROdriver/driver/bmp3.h
    drivers/BAROdriver/driver/bmp3_defs.h
    drivers/BAROdriver/driver/bmp3_selftest.c
    drivers/BAROdriver/driver/bmp3_selftest.h

    drivers/IMUdriver/IMUdriver.cpp
    drivers/IMUdriver/IMUdriver.hpp

    drivers/GPSdriver/GPSdriver.cpp
    drivers/GPSdriver/GPSdriver.hpp

    drivers/SERVOdriver/SERVOdriver.cpp
    drivers/SERVOdriver/SERVOdriver.hpp

    drivers/RXTXdriver/RXTXdriver.cpp
    drivers/RXTXdriver/RXTXdriver.hpp
    )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    hardware_i2c
    hardware_pwm
	)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../lib/FreeRTOS-Kernel/include
    )
     
target_compile_definitions(${NAME} PRIVATE
    FREE_RTOS_KERNEL_SMP=1
    portSUPPORT_SMP=1
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)