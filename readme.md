## Python-native implementation of LcdnLib
This is a basic python-native implementation of LcdnLib. It implements basic Lcdn methods as well as most of the stepper methods. It solely relies on the package `pyserial`.

# Lcdn methods
The following Lcdn methods were implemented.


| Method             | Description                                     |
|--------------------|-------------------------------------------------|
| LdcnHardReset      | Resets all devices (broadcast)                  |
| LdcnSendCmd        | Send command to device                          |
| LdcnInit           | Initialize network using default group 0xFF     |
| LdcnShutdown       | Stop motor, delete devices (reinit necessary)   |
| LdcnGetDeviceId    | Returns `device, devid` for `addr`              |
| LdcnGetDevice      | Returns `device` for `addr`                     |
| LdcnGetStat        | Returns device status                           |
| LdcnGetStatItems   | Returns configured statusitems                  |
| LdcnReadStatus     | Reads local device status                       |


# Stepper methods
The following Stepper methods were implemented.


| Method             | Description                                     |
|--------------------|-------------------------------------------------|
| GetSteppers        | Returns addresses of discovered stepper devices |
| StepStopMotor      | Setup stop behaviour                            |
| StepSetParam       | Sets motor operation parameters                 |
| StepLoadTraj       | Set-up and start movement                       |
| StepResetPos       | Reset motor position to 0                       |

# Usage
To find out how the methods are used, either check LcdnLib documentation or example given in `demo.py`.

# Disclaimer
Usage at your *own risk*, the implementation is *neither complete nor well tested*.