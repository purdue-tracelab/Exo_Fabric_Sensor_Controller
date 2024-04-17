"""
# This is the main launch file for the communication between exoboots and arudino (soft sensor)
# In here, two exoboots need to be presented
"""
import os, sys
thisdir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(thisdir)

from flexsea import flexsea as flex
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe
import serial
import serial.tools.list_ports
# from com_exo_softsensor import *
# from func_com_log_four_sensors import *
# from Class_Arduino import *
from Class_ExoBoots_Arduino import *
import time
# import pandas as p
fxs = flex.FlexSEA()

ZEROING_CURRENT = 400 # mA

def zero_boot (exo) :

	# the gains need to be defined before this is called

	exo.set_controller(fxe.FX_CURRENT)


	exo.set_exo_current(ZEROING_CURRENT)

	time.sleep(2) # wait a second

if __name__ == "__main__":
# def main():
    print('communicating with arduino')
    soft_sensor = Arduino()

    port_cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ports.yaml")
    ports, baud_rate = fxu.load_ports_from_file(port_cfg_path)
    print(f"Using ports:\t{ports}")
    print(f"Using baud rate:\t{baud_rate}")

    # filename = 'C:\\Users\\Trace\\Desktop\\dephy\\Optimization_test\\control_law.xlsx'
    # data = pd.read_excel(filename,usecols=[4])
    # must turn on left boot then right boot.  Otherwise this will break.
    # Let COM 4 to be the left boot, COM 13/14 to be the right boot.
    # (the smaller COM number be the left boot, the larger COM number to be the right boot)
    # TODO: find a better solution for this but currently we don't have access to boot specific IDs.
    idx = 0  # keep idx = 0 for left boot
    print('creating left boot')
    left_boot = ExoBoot(ports[0], int(baud_rate), idx, log_level=4, shouldLog=True,
                        frequency=100, soft_sensor_data = soft_sensor.rawdata2)  # recent change the first line is now the baud rate
    idx += 1  # keep idx = 1 for right boot
    right_boot = ExoBoot(ports[1], int(baud_rate), idx, log_level=4, shouldLog=True, frequency=100, soft_sensor_data = soft_sensor.rawdata3)

    if False:  # ( (not left_boot.id) or (not right_boot.id) ):
        print("Cannot find left boot or arduino")
    else:
        # TODO: Add in gains for current and position
        left_boot.define_current_gains(40, 400, 128)
        zero_boot(left_boot)
        print('starting loop')
        try:
            while True:
                # put main code loop here it will stop with a CTRL+c
                # while ((time.monotonic()-start_time) <= (1/left_boot.frequency)) : # if you run boot interface loop faster than the dephy frequency the cue will fill up and it will take a long time shut down and to have the data come in.  This just waits till the boot period has passed then runs.
                # pass
                # start_time = time.monotonic()
                soft_sensor.obtain_softsensor_data()
                left_boot.read_data(soft_sensor.rawdata2)
                right_boot.read_data(soft_sensor.rawdata3)

        except KeyboardInterrupt:
            print("KeyboardInterrupt has been caught.")

    left_boot.set_exo_current(0)

    time.sleep(.3)

    left_boot.set_controller(fxe.FX_NONE)

    time.sleep(.3)
    del left_boot
    # cleanupPlanStack()

# main()