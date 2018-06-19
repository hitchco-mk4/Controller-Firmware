# Controller-Firmware

## Serial Communication



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
|  unfilled | db_f9 | float | 4 |
|  unfilled | db_f10 | float | 4 |
|  unfilled | db_f11 | float | 4 |
|  Engine Temp | db_i0 | int | 2 |
|  Air Charge Temp | db_i1 | int | 2 |
|  State | db_i2 | int | 2 |
|  unfilled | db_i3 | int | 2 |
|  Emergency Break | db_b0 | byte | 1 |
|  Reverse | db_b1 | byte | 1 |
|  Shutoff | db_b2 | byte | 1 |
|  Left Blinker Status | db_b3 | byte | 1 |
|  Right Blinker Status | db_b4 | byte | 1 |
|  unfilled | db_b5 | byte | 1 |
|  unfilled | db_b6 | byte | 1 |
|  crc | db_b7 | byte | 1 |
