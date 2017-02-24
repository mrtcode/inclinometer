# Inclinometer prototype

Uses Atmega328 + LSM303DLH accelerometer

When turned on calibrates and then starts sending inclination angle (0-180) over USART. Removes gravity vector from accelerometer data automatically.

![](https://mrtcode.github.io/inclinometer/diagram.png)