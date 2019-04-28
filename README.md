# ECE5780_Project
Our nice wheely spinner

Just a place for our code

If you decide to use the Gyroscope and USART, the pinlayout is as follows:

Gyroscope:  

PB13 connected to 10k resistor and 3.3V

PB15 and PB11 connected to the same 10k resistor and 3.3
           
UART:      PA9->RX
           PA10->TX
           
The server ESP should have its RX pin connected to the STM32's TX pin.
The client ESP should have pins 12 and 13 connected to the motor driver's IN1/IN2 pins. 5V and EN pins on the motor driver should be supplied with either Vbat or 3.3V from the ESP.
Additionally, the motor driver must be supplied with at least 5 volts from an external power source connected to the screw terminal. Do not exceed 11 volts. To drive a motor, connect its wires to MTR_BLK and MTR_RED on the motor driver.
