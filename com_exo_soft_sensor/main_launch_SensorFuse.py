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
from Class_ExoBoots_SoftSensor import *
import time
import threading

fxs = flex.FlexSEA()

ZEROING_CURRENT = 200  # mA


def zero_boots(leftExo, rightExo):
    # the gains need to be defined before this is called

    leftExo.set_controller(fxe.FX_CURRENT)
    rightExo.set_controller(fxe.FX_CURRENT)

    leftExo.set_exo_current(ZEROING_CURRENT)
    rightExo.set_exo_current(ZEROING_CURRENT)

    time.sleep(1)  # wait a second


# Function to read data from the soft sensor and calculate average values over 3 seconds
def read_soft_sensor(s_sensor, l_init_data, r_init_data):
    s_sensor.obtain_softsensor_data()
    count = 0
    sensor_data_sums = [0, 0, 0, 0]
    s_time = time.time()
    while time.time() - s_time <= 3:  # Read data for 3 seconds
        for i in range(1, 3):  # Iterate over the sensor data indices
            sensor_data_sums[i - 1] += s_sensor.left_sensor_data[i]
            sensor_data_sums[i + 1] += s_sensor.right_sensor_data[i]
        count += 1
    avg_sensor_data = [s_sum / count for s_sum in
                       sensor_data_sums]  # Calculate averages
    l_init_data = [avg_sensor_data[0], avg_sensor_data[1]]
    r_init_data = [avg_sensor_data[2], avg_sensor_data[3]]
    print('left_init_data: ' + str(l_init_data[0]))
    print('right_init_data: ' + str(r_init_data[0]))


# Function to read data from the exoboot
def read_left_exoboot(l_boot, l_sensor_data, left_i_data):
    # should read left soft sensor and exoboot
    l_boot.read_data(soft_sensor_data=l_sensor_data, sensor_init_data=left_i_data)


def read_right_exoboot(r_boot, r_sensor_data, right_i_data):
    # should read left soft sensor and exoboot
    r_boot.read_data(soft_sensor_data=r_sensor_data, sensor_init_data=right_i_data)


def left_torque_profile(l_boot, l_sensor_data, rspg_, ot_, left_i_data):
    # initialize the Zhang/Collins profile
    l_boot.init_collins_profile(ramp_start_percent_gait=rspg_, onset_torque=ot_)
    # apply the torque appropriate to the Zhang/Collins profile
    l_boot.run_collins_profile(soft_sensor_data=l_sensor_data,
                               sensor_init_data=left_i_data)


def right_torque_profile(r_boot, r_sensor_data, rspg_, ot_, right_i_data):
    # initialize the Zhang/Collins profile
    r_boot.init_collins_profile(ramp_start_percent_gait=rspg_, onset_torque=ot_)
    # apply the torque appropriate to the Zhang/Collins profile
    r_boot.run_collins_profile(soft_sensor_data=r_sensor_data,
                               sensor_init_data=right_i_data)


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
    left_boot = ExoBoot(ports[0], int(baud_rate), idx, log_level=4, shouldLog=True, frequency=100)
    idx += 1  # keep idx = 1 for right boot
    right_boot = ExoBoot(ports[1], int(baud_rate), idx, log_level=4, shouldLog=True, frequency=100)

    if False:  # ( (not left_boot.port) or (not right_boot.port) ):
        print("Cannot find left boot or Arduino")
    else:
        # TODO: Add in gains for current and position
        left_boot.define_current_gains(40, 400, 128)
        right_boot.define_current_gains(40, 400, 128)

        zero_boots(left_boot, right_boot)
        print("-----LOAD CONTROL LAW-----")
        rspg = 0  # ramp start percent gait
        ot = 0.056  # set onset torque to be 0.056 Nmm/ onset current to be 200mA

        print('rspg: ' + str(rspg))
        print('ot: ' + str(ot))
        print('starting loop')


        # Initialize sums for left_sensor_data[1], left_sensor_data[2], right_sensor_data[1], right_sensor_data[2]
        count = 0
        print_count = True
        left_init_data = []
        right_init_data = []
        sensor_data_sums = [0, 0, 0, 0]
        start_time = time.time()
        try:
            while True:
                # read data from soft sensor and exoboots
                soft_sensor.obtain_softsensor_data()
                while (time.time() - start_time >= 1) and (time.time() - start_time <= 4):  # Read data for 3 seconds
                    sensor_data_sums[0] += soft_sensor.left_sensor_data[0]  # left hip
                    sensor_data_sums[1] += soft_sensor.left_sensor_data[1]  # left knee
                    sensor_data_sums[2] += soft_sensor.right_sensor_data[0]  # right hip
                    sensor_data_sums[3] += soft_sensor.right_sensor_data[1]  # right knee
                    count += 1

                if time.time() - start_time > 4:
                    avg_sensor_data = [s_sum / count for s_sum in
                                       sensor_data_sums]  # Calculate averages
                    left_init_data = [avg_sensor_data[0], avg_sensor_data[1]]
                    right_init_data = [avg_sensor_data[2], avg_sensor_data[3]]
                    if print_count:
                        print('left_init_data : ' + str(left_init_data))
                        print('right_init_data: ' + str(right_init_data))
                        print('Ready for next step')
                        print_count = False

                while time.time() - start_time > 5:  # wait for 1 second
                    soft_sensor.obtain_softsensor_data()
                    left_boot_read_thread = threading.Thread(target=read_left_exoboot,
                                                             args=(left_boot, soft_sensor.left_sensor_data, left_init_data))
                    left_boot_read_thread.start()
                    right_boot_read_thread = threading.Thread(target=read_right_exoboot,
                                                              args=(right_boot, soft_sensor.right_sensor_data, right_init_data))
                    right_boot_read_thread.start()

                    # let all threads be finished
                    left_boot_read_thread.join()
                    right_boot_read_thread.join()

                    #
                    left_torque_thread = threading.Thread(target=left_torque_profile,
                                                          args=(left_boot, soft_sensor.left_sensor_data, rspg, ot, left_init_data))
                    left_torque_thread.start()

                    right_torque_thread = threading.Thread(target=right_torque_profile,
                                                           args=(right_boot, soft_sensor.right_sensor_data, rspg, ot, right_init_data))
                    right_torque_thread.start()

                    left_torque_thread.join()
                    right_torque_thread.join()

        except KeyboardInterrupt:
            print("KeyboardInterrupt has been caught.")

    left_boot.set_exo_current(0)

    time.sleep(.3)

    left_boot.set_controller(fxe.FX_NONE)

    time.sleep(.3)
    del left_boot
    del right_boot
    # cleanupPlanStack()

# main()
