_This repo is a part of the Hitchco MK4 project. You can read more about the project [here](http://www.esologic.com/hitchco-mk4/)._

# Controller-Firmware

![software data flow](https://user-images.githubusercontent.com/3516293/39070220-7587746e-44b0-11e8-97f2-8d20e8ba16bc.png)

This is the firmware running on the Arduino Mega attached to the main board. 

It is a very straight forward and minimal state machine design pattern. The most interesting part of the code is the Serial communication with the Raspberry Pi running the display software. 

After some experimentation, I determined that It would probably be a bad idea for a Raspberry Pi to be responsible for both serving a GUI and doing all of the low-level signal processing to make the car work. This firmware takes some commands from the Raspberry Pi but mostly operates independently, sending back the status of the car as needed. The car operate without the Pi being powered on, in the state of a power line disaster that fries the sensitive components on the Pi. 

## Serial Communication

Every time the Arduino moves through the state machine, it processes messages from the Pi. If the Pi is requesting data about the car, the Arduino sends it via Serial3 on the Mega.

A large buffer of 64 bytes is maintained in memory and transmitted to the Pi. Checksums are used to verify the integrity of the transmission.

The following table describes how this 64 bytes is laid out:

|  **Signal Name** | **Variable Name** | **Type** | **# Bytes** |
|  ------ | ------ | ------ | ------ |
|  RPM | db_f0 | float | 4 |
|  MPH | db_f1 | float | 4 |
|  Battery Voltage | db_f2 | float  | 4 |
|  Oil Pressure | db_f3 | float | 4 |
|  Fuel Level | db_f4 | float | 4 |
|  EGOL | db_f5 | float | 4 |
|  EGOR | db_f6 | float | 4 |
|  Miles this trip | db_f7 | float | 4 |
|  Battery Current | db_f8 | float | 4 |
|  Oil Level | db_f9 | float | 4 |
|  unfilled | db_f10 | float | 4 |
|  unfilled | db_f11 | float | 4 |
|  Engine Temp | db_i0 | int | 2 |
|  Air Charge Temp | db_i1 | int | 2 |
|  State | db_i2 | int | 2 |
|  Light Level | db_i3 | int | 2 |
|  Emergency Break | db_b0 | byte | 1 |
|  Reverse | db_b1 | byte | 1 |
|  Shutoff | db_b2 | byte | 1 |
|  Left Blinker Status | db_b3 | byte | 1 |
|  Right Blinker Status | db_b4 | byte | 1 |
|  Clutch Saftey Switch | db_b5 | byte | 1 |
|  Headlights Enabled | db_b6 | byte | 1 |
|  crc | db_b7 | byte | 1 |

I went through a couple of different implementations to get to this solution. The doing this large transfer may not be the fastest, but it ended up being the most minimal. The GUI is updated as a whole, not in parts.