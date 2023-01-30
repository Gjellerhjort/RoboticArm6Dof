# RoboticArm6Dof
A Robot Arm with  6 steppers controlled with a esp32 coded in embeded C.
# ESP32
### Hardware Required
* 1x Development board with any Espressif SoC which features MCPWM peripheral (e.g., ESP32-DevKitC, ESP-WROVER-KIT, etc.)
* 1x USB cable for Power supply and programming
* 6x Hall effect sensors
* 6x Nema 17 steppers
* 6x A4988 stepper driver
* 1x 24v 17A DC powersupply


## Build and Flash Project
Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.
## Troubleshooting
For any technical queries, please open an [issue] (https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
# 6_stepperMega2560
## Build and Flash Project
`avr-gcc -Os -DF_CPU=16000000UL -mmcu=atmega2560 -c -o main.o main.c`

`avr-gcc -mmcu=atmega2560 main.o -o main`

`avr-objcopy -O ihex -R .eeprom main main.hex`

`sudo avrdude -p m2560 -c stk500v2 -P PORT -b 115200 -F -U flash:w:main.hex`

to use monitor `screen PORT`