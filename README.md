# 3-Reaction-Wheels-Self-Balancing

ESP32, MPU6050, Nidec 24H brushless motors, 500 mAh LiPo battery.

Balancing controllers can be tuned remotely over bluetooth.

Example:

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

The same for K2, K3. Send "i", "s".

<img src="/pictures/3-wheel1.jpg" alt="3-Reaction-Wheels-Self-Balancing"/>

<img src="/pictures/Schematic.png" alt="3-Reaction-Wheels-Self-Balancing-Schematic"/>

About schematic:

Battery: 3S1P LiPo (11.1V). 

Buzzer: any 5V active buzzer.

Voltage regulator: any 5V regulator (7805).

All red connections not nescesary for this project! But if you are designing a PCB I recommend making these connections. Maybe I use encoders in the future, you will be able to use the new firmware without any changes.
 
More about this:

https://youtu.be/hX9QkHH4hb4

https://youtu.be/LR9i28_jxpo

You can also make this with Arduino nano controller. All other parts remain the same.

<img src="/pictures/arduino_schematic.png" alt="3-Reaction-Wheels-Self-Balancing-Schematic"/>

In this version I make offsets setting procedure more simple. First connect to controller over bluetooth. 
You will see a message that you need to calibrate the balancing point. Send c+ from serial monitor. This activate calibrating procedure. 
Set the robot to balancing point. Hold still when the robot does not fall to either side. Send c- from serial monitor. 
This will write the offsets to the EEPROM. After calibrating, the robot will begin to balance.

