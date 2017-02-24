# Inclinometer prototype for a soil probe

Informs an operator about abnormal probe angle to protect it from damage.

Uses Atmega328, LSM303DLH accelerometer and MAX485 transceivers for a long distance communication.

When turned on calibrates and starts sending inclination angle (0-180) over USART. Removes gravity vector from accelerometer data automatically.

![](https://mrtcode.github.io/inclinometer/diagram.png)