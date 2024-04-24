"""
# This is the main launch file for the communication between exoboots and arudino (soft sensor)
# In here, two exoboots need to be presented
"""
import os
import sys

thisdir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(thisdir)

from flexsea import flexsea as flex
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe
from Class_ExoBoots_Arduino_Hip_Phase import *
import time

# import pandas as p
fxs = flex.FlexSEA()

ZEROING_CURRENT = 200  # mA


def zero_boots(leftExo, rightExo):
    # the gains need to be defined before this is called

    leftExo.set_controller(fxe.FX_CURRENT)
    rightExo.set_controller(fxe.FX_CURRENT)

    leftExo.set_exo_current(ZEROING_CURRENT)
    rightExo.set_exo_current(ZEROING_CURRENT)

    time.sleep(1)  # wait a second


if __name__ == "__main__":
    # def main():
    print('communicating with Arduino')
    soft_sensor = Arduino()

    port_cfg_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ports.yaml")
    ports, baud_rate = fxu.load_ports_from_file(port_cfg_path)
    print(f"Using ports:\t{ports}")
    print(f"Using baud rate:\t{baud_rate}")

    # must turn on left boot then right boot.  Otherwise this will break.
    # Let COM 16 to be the left boot, COM 20 to be the right boot.
    # (the smaller COM number be the left boot, the larger COM number to be the right boot)
    # TODO: find a better solution for this but currently we don't have access to boot specific IDs.
    idx = 0  # keep idx = 0 for left boot
    print('creating left boot')
    left_boot = ExoBoot(ports[0], int(baud_rate), idx, log_level=4, shouldLog=True,
                        frequency=100, soft_sensor_data=soft_sensor.left_sensor_data)  # recent change the first line is now the baud rate
    idx += 1  # keep idx = 1 for right boot
    right_boot = ExoBoot(ports[1], int(baud_rate), idx, log_level=4, shouldLog=True,
                         frequency=100, soft_sensor_data=soft_sensor.right_sensor_data)

    if False:  # ( (not left_boot.id) or (not right_boot.id) ):
        print("Cannot find left boot or Arduino")
    else:
        # TODO: Add in gains for current and position
        left_boot.define_current_gains(40, 400, 128)
        right_boot.define_current_gains(40, 400, 128)

        zero_boots(left_boot, right_boot)

        print("-----LOAD CONTROL LAW-----")
        # user_mass = 75
        rspg = 0
        # opg = 23.1
        # ppg = 48.4
        # spg = 53  # 60
        ot = 0.056  # set onset current to be 200mA
        # npt = 0.025 / 4  # set peak current to be optimized one

        # print('user mass: ' + str(user_mass))
        print('rspg: ' + str(rspg))
        # print('opg: ' + str(opg))
        # print('ppg: ' + str(ppg))
        # print('spg: ' + str(spg))
        print('ot: ' + str(ot))
        # print('npt: ' + str(npt))

        print('starting loop')

        try:
            while True:
                # read data from soft sensor and exoboots
                soft_sensor.obtain_softsensor_data()
                left_boot.read_data(soft_sensor_data=soft_sensor.left_sensor_data)  # should read left hip sensor
                right_boot.read_data(soft_sensor_data=soft_sensor.right_sensor_data)  # should read right hip sensor

                left_boot.init_collins_profile(ramp_start_percent_gait=rspg, onset_torque=ot)  # initialize the Zhang/Collins profile
                right_boot.init_collins_profile(ramp_start_percent_gait=rspg, onset_torque=ot)  # initialize the Zhang/Collins profile

                # apply the torque appropriate to the Zhang/Collins profile
                left_boot.run_collins_profile(soft_sensor_data=soft_sensor.left_sensor_data)  # should read left hip sensor
                right_boot.run_collins_profile(soft_sensor_data=soft_sensor.right_sensor_data)  # should read right hip sensor

                # time.sleep(1 / left_boot.frequency)  # Lazy alternative to the waiting while loop but will run a bit slower.

        except KeyboardInterrupt:
            print("KeyboardInterrupt has been caught.")

    left_boot.set_exo_current(0)

    time.sleep(.3)

    left_boot.set_controller(fxe.FX_NONE)

    time.sleep(.3)
    del left_boot
    # cleanupPlanStack()

# main()
