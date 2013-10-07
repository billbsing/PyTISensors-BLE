PyTISensors-BLE
===============

Python TI Sensors for Bluetooth Low Energy


pyTISensorsDemo.py
------------------

This script allows you to try out the different Texas Instruments Sensors.
When you connect to a sensor the script will automatically load in the correct
sensor software and communicate to the sensor via the GATTServer module.

To talk to the TI sensors you need to have the TI bluetooth development USB dongle connected to your computer.
	

This repo also contains a GATTServer. This server handles all GATT and ATT protocols between the demo script and the 
bluetooth USB dongle.

