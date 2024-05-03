# Exo_Fabric_Sensor_Controller
This is the exoboot controller that fuses the exoboots and fabric sensor (hip+knee).

## Before using this branch
Make sure you have a connection between PC and the soft sensor. 

### DAQ of soft sensor:
More details are in the branch `com_hip_sensor`.
1. Current use (built by UML NERVE team): check the `soft_sensor_daq` folder, and find the `sensor driver documentation.pdf` to see the current hardware setup. 

   In the `sensor drive documentation.pdf` file, the project box includes the rubber sensor driver, which is no longer used.
   
   <img src="soft_sensor_daq/hardware_pics/old_project_box.jpg" width="300">
   
   Load `MPR121_driver_BodySuit_test.ino` to the Arduino board. This file is from The Faboratory lab at Yale University [url](https://www.eng.yale.edu/faboratory/).
   
   To log soft sensor data alone, you can use `com_log_four_sensors.py` file.
   Make sure all FOUR sensors are connected, and the Python script will read data from pin 0 to pin 3.

   In this branch, pin 0  is left hip sensor; pin 1 is left knee; pin 2 is right hip; pin3 is right knee.

### Communication between soft sensor and exoboots
The soft sensors are not directly communicating with exoboots (not send sensor data to the exoboot board). Instead, the soft sensor readings are sent to the PC via USB cable, as well as exoboot readings. The detailed logic is written in the `main_launch_SensorFuse.py` file.

## This branch is used for replacing manually tuned parameters with soft sensor readings
1. Use the maximum hip extension (MHE) time to replace the peak time.
2. Use the maximum hip extension (MHE) value to replace the normalized peak torque parameter.
3. The rise/fall times are proportional to the peak time.
4. The heel strike event is detected by knee soft sensor readings.

## Reference
If the MoCap system is used to check the accuracy of the gait event estimation, use [Rizzoli Markersets](https://v23.wiki.optitrack.com/index.php?title=Rizzoli_Markersets#Rizzoli_Body_Protocol.2837.29) for the marker placement on the subject.

## Contribution
(Internal usage only)

Zenan Zhu, [TRACE lab](https://www.thetracelab.com/)

