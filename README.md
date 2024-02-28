<div align="center">

# FCP - autonomous Flight Controller Project
This repository hosts the source code and documentation for an autonomous flight controller project,<br>
designed to enable unmanned aerial vehicles (UAVs) to navigate and perform tasks independently.<br>
Leveraging Low-Budget hardware and Open Source software, this project aims to empower enthusiasts and researchers<br>
to dive into the field of autonomous flight and develop<br>
another forms of autonomous flight controller software based on this project.

</div>

<div>

## Hardware
+ [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)
+ [Adafruit BNO055](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)
+ [Adafruit Ultimate V3 GPS](https://learn.adafruit.com/adafruit-ultimate-gps/overview)
+ [Adafruit BMP388](https://learn.adafruit.com/adafruit-bmp388-bmp390-bmp3xx)
+ [FlySky i6x](https://www.flysky-cn.com/fsi6x)
+ [FlySky FS-iA6B](https://www.flysky-cn.com/ia6b-canshu)
+ Sky Surfer RC Plane (1400mm)
+ SunnySky X2212 iii - 2450KV
+ HobbyWing SkyWalker ESC (50a with 5V BEC)
+ 3S Lipo Battery
+ regular 9g servos
+ electronic diy hardware (breadboard, jumper wires, resistors and more)

## Dependencies
+ [FreeRTOS Kernel](https://github.com/FreeRTOS/FreeRTOS-Kernel) Version V202110.00-SMP
+ Raspberry Pi Pico C/C++ SDK

</div>

<div>

## Build process
```
$ cd FCP
$ mkdir build
$ cd build
$ cmake ..
$ make
```
Copy UF2 binary file to the pico.<br>
You will also need to change flight parameters in main file (lines 28, 50).

</div>
