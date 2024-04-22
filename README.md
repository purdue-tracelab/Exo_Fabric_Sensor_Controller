# Exo_Fabric_Sensor_Controller
This is the exoboot controller that fuses the exoboots and fabric sensor.

To run the exo controller based on fabric sensor inputs, you need to first look into the Dephy actuator package and install their `FlexSEA` library.

## Dephy Actuator Package
Dephy exoboots website: 
https://dephy.com/start/

Dephy exoboots version: 
EB604, ActPack 4.1

For battery information: https://dephy.com/faster/battery/BA30/C_0002_DS_0001_V03_BA30DATA.pdf

## How to actuate Dephy Exoboots
### Depending on different purchase options of Dephy Exoboots, you may have:
1. `2021-08-24-Plan-WalkOnly-Win-32-v7.2.0`: New plan 4.0 GUI for Dephy controller mode (as we don't have the default Dephy controller, this one is not necessary to use)
3. `Actuator-Package-7.2.0`: New API package for custom controller mode (the main folder used here)
4. `dephy_bootloader`: The boot loader for switching between two controller modes
   (as we don't have the default Dephy controller, this one is not necessary to use)

### Exoboots have two controller modes: Dephy default controller mode and custom controller mode
1. Details on the Dephy website
2. To switch the controller mode, you need to follow the Dephy boot loader guide
3. Dephy boot loader guide: `C_0004_UG_0001_V01_BOOTLOADGUIDE`

### To start with a custom controller: 
1. Locate the `Getting Started with the ActPack` in the start website
2. Download API package from: https://github.com/DephyInc/Actuator-Package
3. Find Tags `v7.2.0` (or the latest stable version of API package) and download the API package
4. Follow the `README.md` in the GitHub website and install the Install `FlexSEA` Library 
5. To install `MinGW` on Windows 10 x64,  use `MinGW-W64` Online Installer first. Select the options as `x86_64`, `posix`, `seh`
6. Get the `x86_64-posix-seh` file if `MinGW` cannot be installed properly by the exe file. Search YouTube videos to install `MinGW` and add the path to the system.

If the `FlexSEA` Library is installed successfully, try to run the demo script by connecting the exoboots with a USB cable, but DO NOT put the battery in the boots.	
1. Demo scripts of python: `..\Actuator-Package-7.2.0\Python\flexsea_demo`, run the python script `run_demos.py` and select `0 read only` for testing
2. Details in `3 GUI and Demo Scripts` in the Dephy website
3. You can check the data log file in `DataLog` folder.
4. Check data units in 13 `Units` on the Dephy website
5. To communicate with Exoboots, the ports that are used to connect to the boots need to be found. Modify the `port.yaml` file before running any Python scripts
6. Details in 4.1.1“Pair ExoBoot via Bluetooth (Windows)” on the Dephy website

To get familiar with Dephy API, read demo scripts (i.e., current_control.py) with API definition files that are located in `..\Actuator-Package-7.2.0\Python\flexsea\flexsea`
1. One API example: `dev_id = fxs.open(port, baud_rate, log_level=6)`
2. `Open()` is a function defined in `Flexsea` definition file 
3. Details in `3.2 Demo Scripts` in Dephy website
4. The API link in the first paragraph of the 3.2 section is the old API functions; DO NOT read those functions. Use the definition files located in the `..\Actuator-Package 7.2.0\Python\flexsea\flexsea`.
5. For data variables that exoboots give, read the “ActPackState.py” in `..\Actuator-Package-7.2.0\Python\flexsea\flexsea\dev_spec`

Future Work after running the demo scripts:

For the human-in-the-loop basic controller, put the `basic_controller` folder of this GitHub repo under `..\Actuator-Package 7.2.0\Python\` in your PC.
1. To run the controller, run `test_1_law.py`. Before that, you need to connect to TWO boots and change the COM ports in the `port.yaml` file
2. To run the controller with ONE boot, modify `test_1_law_left_boot.py` and run it. The modification of the `test_1_law_left_boot.py` script has not been completed yet; it still needs to be modified. Before running the script, you still need to change the `port.yaml` file.
3. The main controller definition is in `test_new_def_v2.py` file. Understand each function in the file before running the controller.
4. The “test_old_def.py” file contains the old `dephy` API. You can compare two scripts (old and new API) to see the differences.

## com_exo_soft_sensor 
Use this folder for the sensor fusion of exo and soft sensors. See other branches for details.

### Do not use com_exo_soft_sensor folder before testing Dephy Exoboots!
