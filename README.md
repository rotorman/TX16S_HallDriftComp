# RadioMaster TX16S Hall stick drift compensation

Test software for the RadioMaster TX16S hall sticks. Using [ST LSM6DS33](https://www.st.com/en/mems-and-sensors/lsm6ds33.html) as a temperature reference.

An example hookup of [Adafruit LSM6DS33 breakout board](https://www.adafruit.com/product/4480) connected to TX16S AUX1:

<img src="media/TX16S_LSM6DS33_hookup.jpg">

The source code is to be used with [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html). It reconfigures the original serial port USART3 at AUX1 on STM32F429BIT6 to I2C bus (I2C2) and enables the 5V power output of AUX1. It samples all 4 stick channels and slider 1 as a non-hall reference as well. Debug output is available at AUX2/USART6 with 400.000 baud 8N1.

Hookup between RadioMaster TX16S AUX1 port and Adafruit LSM6DS33 breakout board:

* TX16S AUX1 RX - Adafruit LSM6DS33 SDA
* TX16S AUX1 TX - Adafruit LSM6DS33 SCL
* TX16S AUX1 5V - Adafruit LSM6DS33 VIN
* TX16S AUX1 GND - Adafruit LSM6DS33 GND

Via TX16S AUX2/USART6 you can expect human readable periodic output of the IMU and hall sticks (raw 12-bit ADC value). I include in the output also the core temperature of the STM32F429 integrated temperature sensor and Slider1 as non-hall reference. Here is an example line on the second row of the debug output:
```
STM32_Temp;Temp;AccX;AccY;AccZ;GyroX;GyroY;GyroZ;LH;LV;RH;RV;Slider1
33.17;24.93;2.02;-4.17;8.92;0.05;-0.11;-0.04;2180;3290;2080;2106;2041
```
The units are temperature in degrees Celsius, linear acceleration in m/s<sup>2</sup>, angular rate in rad/s, sticks in unitless 12-bit ADC raw output.

A brief press on the power button turns on the radio, a press longer than 1 second, turns the radio off again.
