PyTISensors-BLE
===============

Python TI Sensors for Bluetooth Low Energy


TISensorsDemo
-------------

This script allows you to try out the different Texas Instruments Sensors.
When you connect to a sensor the script will automatically load in the correct
sensor software and communicate to the sensor via the GATTServer module.

To talk to the TI sensors you need to have the TI bluetooth development USB dongle connected to your computer.

This script can communicate automatically to the TI Sensor Tag and to the TI KeyFob.



Usage:


    usage: pyTISensorsDemo.py [-h] [--device DEVICE] [--timeout TIMEOUT]
                              [--address ADDRESS] [-v]
                              [command] [options [options ...]]
    
    Talk Bluetooth to CC2540/1 Sensors
    
    positional arguments:
      command               Possible commands are: 
                            discover: List known devices within range.
                            accelerometer: Return accelerometer information.
                            gyroscope: Return gyroscope information.
                            temperature: Returns the object and ambient temperature.
                            humidity: Returns the temperature and humidity. 
                            magnetometer: Returns the magnetic position.
                            barometer: Returns the temperature and pressure.
      options               options
    
    optional arguments:
      -h, --help            show this help message and exit
      --device DEVICE, -d DEVICE
                            Device to connect too
      --timeout TIMEOUT, -t TIMEOUT
                            Timeout in seconds before a GATT command will fail.
      --address ADDRESS, -a ADDRESS
                            Address of device to connect to usually returned by
                            the discover command. If not set then a discovery will
                            be made and the first device on the list will be used.
      -v, --verbose         Show more messages
    


This repo also contains a GATTServer. This server handles all GATT and ATT protocols between the demo script and the 
bluetooth USB dongle.

