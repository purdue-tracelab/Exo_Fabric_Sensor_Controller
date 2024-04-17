"""
# This is the class for the communication between Exoboots and Arduino (soft sensor)
"""
import os
import sys
import platform
import serial
import serial.tools.list_ports

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from flexsea import flexsea as flex
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe

from collections import deque
import math as m
from datetime import datetime
import configparser
import time

fxs = flex.FlexSEA()
# set current limit to 1A for baseline test
CURRENT_LIMIT = 4500  # mA : Current limit for the motor command.  You should still set the values in boots protection circuit because the PI controller and position controller can still push it higher.

QUE_LENGTH = 50  # number of samples to use for the LSTM inferance.

MAX_FILE_SIZE = 500 * 10 ** 6  # 500 MB : limits the file size writting so you don't have to open a really big file

NUM_GAIT_TIMES_TO_AVERAGE = 4  # for the gait duration estimate how many gait cycles to average.  More changes slower but is more stable.  This might become obsolete if we move to an infinate horizon filter.

ARMED_DURATION_PERCENT = 10

gyro_conversion_factor = 32.8

NO_SLACK_CURRENT = 400
# NO_SLACK_CURRENT = 400


# convert degrees to radians
def DEG_TO_RAD(angle_deg):
    return (angle_deg) * m.pi / 180


# convert mNm or Nmm to Nm
def NMM_TO_NM(torque):
    return (torque) / 1000


# convert Nm to mNm or Nmm
def NM_TO_NMM(torque):
    return (torque) * 1000


# convert A to mA
def A_TO_MA(torque):
    return (torque) * 1000


# convert mA to A
def MA_TO_A(torque):
    return (torque) / 1000


"""
# Class to interface with the exoboot
"""
class ExoBoot:

    #
    # Initialize the instance
    # inputs:
    # 	devSide		: see values above or use LEFT or RIGHT
    #	port		: port to use for communication
    #	baudRate	: Baud rate for the serial communication with the boot
    #	idx			: Idx of the port
    #	frequency	: Update frequency for the boot (how often it puts data in the buffer
    #	shouldLog	: Should the instance write data to a csv file
    #	shouldAuto	: Should the boot automatically add data to the buffer (true), if false it will only do that when requested.
    #	mode		: What pin refrence is used "BOARD" is more compatable between the RPi and other boards
    #	use_tactor	: Should a tactor be used.  This is convenient to make false if you just want to test the boots without the peripheral board.
    #
    def __init__(self, port, baud_rate, idx, log_level, shouldLog, frequency, soft_sensor_data):
        # Attach the values to the instance.

        print('boot constructor : top')
        self.port = port
        self.baud_rate = baud_rate
        self.idx = idx
        # self.side = devSide #no longer needed for new boots

        self.log_level = log_level
        self.frequency = frequency
        self.should_log = shouldLog
        # self.should_auto = shouldAuto

        self.mode = None

        # Values used for zeroing the encoder position
        # self.motor_ticks_offset = 0
        # self.ankle_ticks_offset = 0

        # Zhang/Collins parameters
        self.t0 = -1  # ramp start percent
        self.t1 = -1  # ramp end percent
        self.t2 = -1  # peak percent
        self.t3 = -1  # drop percent
        self.ts = -1  # top torque for the ramp
        self.tp = -1  # peak torque
        self.user_mass = -1  # user mass
        self.peak_torque_normalized = -1  # torque normalized to user mass

        # parameters for the Zhang/Collins torque curve
        self.a1 = -1
        self.b1 = -1
        self.c1 = -1
        self.d1 = -1

        self.a2 = -1
        self.b2 = -1
        self.c2 = -1
        self.d2 = -1

        self.segmentation_trigger = False  # goes high when heelstrike detected
        self.heelstrike_armed = False  # high when trigger armed

        # for new exoboots, the gyro reading is in bits, need to convert to deg
        self.segmentation_arm_threashold_raw = 150 * gyro_conversion_factor  # the threashold that must be gone above to arm the trigger
        #self.segmentation_arm_threashold = 150   # the threashold that must be gone above to arm the trigger
        self.segmentation_trigger_threashold = 0  # the theashold that must be dropped below to trigger the heelstrike

        self.past_gait_times = [-1] * NUM_GAIT_TIMES_TO_AVERAGE  # store the most recent gait times

        self.expected_duration = -1  # current estimated gait duration

        self.heelstrike_timestamp_current = -1  # Timestamp of the most recent heelstrike
        self.heelstrike_timestamp_previous = -1  # Timestamp of the second most recent heelstrike
        self.armed_timestamp = -1  # timestamp of wsegmentation_trigger_threasholdhen the trigger was armed
        self.percent_gait = -1  # estimate of the percent of gait
        self.percent_gait_previous = -1  # the previous loops estimate of the percent gait. used to determine crossing of timepoints

        self.current_cmd = None  # how much current are we requesting

        # get the labels of the values we are streaming from the boot
        self.labels_stream = ["State time", \
                              "Accel X", "Accel Y", "Accel Z", \
                              "Gyro X", "Gyro Y", "Gyro Z", \
                              "Encoder angle", "Encoder velocity", "Encoder acceleration", \
                              "Motor voltage", "Motor current", \
                              "Battery voltage", "Battery current", \
                              "Ankle angle", "Ankle vel", "System time"
                              ]

        # store the index values for accessing the specific variables
        self.idx_time = 0
        self.idx_accl_x = 1
        self.idx_accl_y = 2
        self.idx_accl_z = 3
        self.idx_gyro_x = 4
        self.idx_gyro_y = 5
        self.idx_gyro_z = 6
        self.idx_motor_angle = 7
        self.idx_motor_vel = 8
        self.idx_motor_accl = 9
        self.idx_motor_voltage = 10
        self.idx_motor_current = 11
        self.idx_batt_voltage = 12
        self.idx_batt_current = 13
        self.idx_ankle_angle = 14
        self.idx_ankle_vel = 15
        self.idx_system_time = 16
        self.idx_other_base = 17
        # Control system info
        # self.idx_sync = self.idx_ankle_vel + 1
        self.idx_percent_gait = self.idx_other_base
        self.idx_heelstrike_armed = self.idx_other_base + 1
        self.idx_segmentation_trigger = self.idx_other_base + 2
        self.idx_expected_duration = self.idx_other_base + 3
        self.idx_current_cmd = self.idx_other_base + 4
        self.idx_torque_cmd = self.idx_other_base + 5
        # soft sensor info
        self.idx_soft_sensor = self.idx_other_base + 6

        # communicate with exoboots
        print('boot constructor : connecting to device')
        self.dev_id = self._connectToDevice()  # connect to the boot
        fxs.start_streaming(self.dev_id, freq = self.frequency, log_en = self.should_log)
        time.sleep(0.4)

        # the labels of the other values we are writing that don't come from the boot
        # the label of the soft sensor value as well
        self.labels_other = [
            "Percent Gait", "Heelstrike Armed", "Segmentation Trigger", "Expected Duration", "Current Command",
            "Torque Command", "Hip Soft Sensor"]

        # the label of the soft sensor value
        # self.labels_soft_sensor = []

        # create a deque that the LSTM will use for its inference.
        self.data_que = deque(maxlen=QUE_LENGTH)
        self.data_exo = [-1] * len(self.labels_stream)  # fill in the data with -1 till we read in the values
        self.data_other = [-1] * len(self.labels_other)  # fill in the data with -1 till we read in the values
        self.labels_current = self.labels_stream + self.labels_other# combine the labels
        self.data_current = self.data_exo + self.data_other # combine the data

        self.data_file = None  # store the data file handle
        if self.should_log:  # if we are logging create the file
            self.file_base = ""
            self.file_extension = ""
            self.data_filename = ""

            # initilize the logging
            self.data_file = self.log_init()

        self.read_data(soft_sensor_data)  # read in the current data from the system

        self.data_current = [0] * len(
            self.labels_stream)  # clear this variable since the first read will not have all the zeroing set
        self.data_que.clear()  # clear this variable since the first read will not have all the zeroing set

        self.kt = 140  # motor torque constant Nmm/A 140 is the torque constant given by Dephy.

        # controller properties
        self.currentKp = 0
        self.currentKi = 0
        self.currentff = 0


        #
        # this is pulled from the Dephy examples
        #
    def _connectToDevice(self):
        devIds = fxs.open(self.port, self.baud_rate, self.log_level)

        time.sleep(0.2)
        MAX_DEVICE_ID_ATTEMPTS = 10
        num_attempts = 0
        # need to change here, fxGetDeviceIds() is not the new api
        get_devIds = fxs.get_ids()
        while (num_attempts < MAX_DEVICE_ID_ATTEMPTS and (len(get_devIds) == 0 or len(get_devIds) < (self.idx + 1))):
            # added in a check because it was only recognizing the first idx when running it in a script.  When manually running it would error the first time but be ok the second time.
            time.sleep(0.2)
            get_devIds = fxs.get_ids()
            print("attempt number : " + str(num_attempts))
        # print("devIds : ")
        # print(get_devIds)
        if len(get_devIds) == 0:
            raise Exception('Failed to get device Id')
        dev_id = get_devIds[self.idx]
        print("Devid is: ", dev_id)
        return dev_id

    """
    #
    # rotate two imu data to be consistent
    #
    def rotate_imu(self, accl_x_raw, accl_y_raw, accl_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw):
        # angleOffset = 0
        angleOffset = m.pi
        rotAngleY = 0

        # rotate about the Y axis (which is pointing up) to get
        # the x axis forward, z axis points to the left of the boot

        if (self.idx == 1):
            rotAngleY = angleOffset

        else:
            rotAngleY = 0

        acclX = (accl_x_raw * m.cos(rotAngleY)
                + accl_z_raw * m.sin(rotAngleY))
        acclY = accl_y_raw
        acclZ = (accl_x_raw * -m.sin(rotAngleY)
                + accl_z_raw * m.cos(rotAngleY))

        gyroX = (gyro_x_raw * m.cos(rotAngleY)
                + gyro_z_raw * m.sin(rotAngleY))
        gyroY = gyro_y_raw
        gyroZ = (gyro_x_raw * -m.sin(rotAngleY)
                + gyro_z_raw * m.cos(rotAngleY))

        return [acclX, acclY, acclZ, gyroX, gyroY, gyroZ]
    """
    """
    # this zero encoder may not be used for new api?
    #
    #	Record the motor and ankle position when the function is called
    #
    def zero_encoders(self):
        self.read_data()
        self.motor_ticks_offset = self.motorTicksRaw
        self.ankle_ticks_offset = self.ankleTicksRaw


    #
    #	Update the current sensor information from the system
    #
    """
    def read_data(self, sensor_data):

        act_pack = fxs.read_device(self.dev_id)
        self.data_exo = [act_pack.state_time,
                         act_pack.accelx, act_pack.accely, act_pack.accelz,
                         act_pack.gyrox, act_pack.gyroy, act_pack.gyroz,
                         act_pack.mot_ang, act_pack.mot_vel, act_pack.mot_acc,
                         act_pack.mot_volt, act_pack.mot_cur,
                         act_pack.batt_volt, act_pack.batt_curr,
                         act_pack.ank_ang, act_pack.ank_vel,
                         act_pack.SystemTime]
        if self.data_current[self.idx_time] != self.data_exo[self.idx_time]:  # update the data if new data has come in
            """
            # no longer needed for new exoboots
            # !!! Make sure signs are correct.
            self.motorTicksRaw = self.data_exo[self.idx_motor_angle]
            self.motorTicksZeroed = self.side * (
                        self.motorTicksRaw - self.motor_ticks_offset)  # remove the offset, and adjust for side

            self.ankleTicksRaw = self.data_exo[self.idx_ankle_angle]
            self.ankleTicksZeroed = self.side * (
                        self.ankleTicksRaw - self.ankle_ticks_offset)  # remove the offset, and adjust for side

            accl_gyro_rotated = self.rotate_imu(self.data_exo[self.idx_accl_x], self.data_exo[self.idx_accl_y],
                                                self.data_exo[self.idx_accl_z], self.data_exo[self.idx_gyro_x],
                                                self.data_exo[self.idx_gyro_y], self.data_exo[self.idx_gyro_z])
            """
            self.data_current = [-1] * len(self.data_exo)  # clear it.  I will be extending the data_other to the end and I don't want issues.

            self.data_current[self.idx_time] = self.data_exo[self.idx_time]
            # self.data_current[self.idx_accl_x] = accl_gyro_rotated[0]
            # self.data_current[self.idx_accl_y] = accl_gyro_rotated[1]
            # self.data_current[self.idx_accl_z] = accl_gyro_rotated[2]
            # self.data_current[self.idx_gyro_x] = accl_gyro_rotated[3]
            # self.data_current[self.idx_gyro_y] = accl_gyro_rotated[4]
            # self.data_current[self.idx_gyro_z] = accl_gyro_rotated[5]
            self.data_current[self.idx_accl_x] = self.data_exo[self.idx_accl_x]
            self.data_current[self.idx_accl_y] = self.data_exo[self.idx_accl_y]
            self.data_current[self.idx_accl_z] = self.data_exo[self.idx_accl_z]
            self.data_current[self.idx_gyro_x] = self.data_exo[self.idx_gyro_x]
            self.data_current[self.idx_gyro_y] = self.data_exo[self.idx_gyro_y]
            self.data_current[self.idx_gyro_z] = self.data_exo[self.idx_gyro_z]
            self.data_current[self.idx_motor_angle] = self.data_exo[self.idx_motor_angle]
            self.data_current[self.idx_motor_vel] = self.data_exo[self.idx_motor_vel]
            self.data_current[self.idx_motor_accl] = self.data_exo[self.idx_motor_accl]
            self.data_current[self.idx_motor_voltage] = self.data_exo[self.idx_motor_voltage]
            self.data_current[self.idx_motor_current] = self.data_exo[self.idx_motor_current]
            self.data_current[self.idx_batt_voltage] = self.data_exo[self.idx_batt_voltage]
            self.data_current[self.idx_batt_current] = self.data_exo[self.idx_batt_current]
            self.data_current[self.idx_ankle_angle] = self.data_exo[self.idx_ankle_angle]
            self.data_current[self.idx_ankle_vel] = self.data_exo[self.idx_ankle_vel]
            self.data_current[self.idx_system_time] = self.data_exo[self.idx_system_time]

            self.data_other = [-1] * len(self.labels_other)

            self.check_for_heelstrike()

            if self.segmentation_trigger:
                self.heelstrike_timestamp_previous = self.heelstrike_timestamp_current
                self.heelstrike_timestamp_current = self.data_current[self.idx_time]
                self.update_expected_duration()

            self.percent_gait_previous = self.percent_gait
            self.percent_gait_calc()

            self.torque_cmd = (self.current_cmd / 1000 * self.kt if self.current_cmd != None else None)

            # Gait Estimation Data
            self.data_other[self.idx_percent_gait - self.idx_other_base] = self.percent_gait
            self.data_other[self.idx_heelstrike_armed - self.idx_other_base] = (1 if self.heelstrike_armed else 0)
            self.data_other[self.idx_segmentation_trigger - self.idx_other_base] = (1 if self.segmentation_trigger else 0)
            self.data_other[self.idx_expected_duration - self.idx_other_base] = self.expected_duration
            self.data_other[self.idx_current_cmd - self.idx_other_base] = self.current_cmd
            self.data_other[self.idx_torque_cmd - self.idx_other_base] = self.torque_cmd
            self.data_other[self.idx_soft_sensor - self.idx_other_base] = sensor_data

            self.data_current.extend(self.data_other)

            if self.should_log:
                self.log()

            self.data_que.append(self.data_current)


    def log_init(self):
        start_time = datetime.now()
        time_str = start_time.strftime("%Y_%m_%d_%Hh%Mm")
        side_str = "_left" if self.idx == 0 else "_right"

        # you need to initilize these before calling these
        self.file_base = time_str + side_str
        self.file_extension = ".csv"

        self.data_filename = self.get_free_filename(self.file_base, self.file_extension)

        data_file = open(self.data_filename, 'a')

        labels_csv = ""
        for label in self.labels_current:
            labels_csv += label + ", "
        data_file.write(labels_csv)
        data_file.write("\n ")

        return data_file


    def get_free_filename(self, base, extension):
        data_filename = base + extension
        i = 0
        while os.path.exists(data_filename):
            i += 1
            data_filename = base + "_" + str(i) + extension
        return data_filename


    def log(self):
        if self.should_log:
            # check file size
            if os.path.getsize(self.data_filename) > MAX_FILE_SIZE:
                self.data_file.close()
                self.data_filename = self.get_free_filename(self.file_base, self.file_extension)
                self.data_file = open(self.data_filename, 'a')

                labels_csv = ""
                for label in self.labels_current:
                    labels_csv += label + ", "
                self.data_file.write(labels_csv)
                self.data_file.write("\n ")

            for i in range(0, len(self.data_current)):
                # write out the data.  If the type is None change to 'nan' so it is easy for matlab to parse.
                self.data_file.write(str(self.data_current[i]) if self.data_current[i] is not None else 'nan')
                self.data_file.write(",")
            self.data_file.write("\n ")
        else:
            print("exo_defs :: log(self) : \n\tYOU WANTED TO LOG VALUES BUT LOGGING DATA WAS NOT SELECTED")


    def ankle_torque_to_current(self, torque):
        # get the current based on the torque cmd and the system state
        current = torque / self.kt

        return current


    # ankle angle conversion in here is not needed.
    def ticks_to_angle(self, ticks):
        return ticks * TICKS_TO_ANGLE_COEFF


    def define_current_gains(self, kp, ki, ff):  # for impedance also take in the stiffness and damping
        self.currentKp = kp
        self.currentKi = ki
        self.currentff = ff


    # check the api that modified in here
    def set_controller(self, controlMode):
        self.mode = controlMode  # store the mode so we can easily check what is set as the Dephy lib doesn't return anything to know the current mode
        fxs.send_motor_command(self.dev_id, fxe.FX_NONE,0)  # change the control mode to the one requested,  THIS MUST BE DONE BEFORE THE GAINS ARE SENT TO THE EXO

        if self.mode == fxe.FX_NONE:
            self.current_cmd = None
            fxs.set_gains(self.dev_id, 0, 0, 0, 0, 0, 0)  # clear the gains
            fxs.send_motor_command(self.dev_id, fxe.FX_NONE, 0)

        elif self.mode == fxe.FX_CURRENT:
            self.current_cmd = 0
            self.set_exo_current(0)  # set the current to zero to avoid surprises
            fxs.set_gains(self.dev_id, self.currentKp, self.currentKi,
                      0, 0, 0, self.currentff)  # set the gains for the current control

        else:
            self.current_cmd = None
            fxs.set_gains(self.dev_id, 0, 0, 0, 0, 0, 0) # clear the gains
            fxs.send_motor_command(self.dev_id, fxe.FX_NONE, 0)


    #
    #	Sends the current command accounting for side, positive is plantar flexion
    #

    # check the api that modified in here
    def set_exo_current(self, currentCommand):
        if abs(currentCommand) < CURRENT_LIMIT:
            self.current_cmd = currentCommand
            # setMotorCurrent(self.dev_id, self.side*currentCommand);  # set the current on the exo
            fxs.send_motor_command(self.dev_id, fxe.FX_CURRENT, currentCommand)
        else:
            print("exoBoot :: set_exo_current : CURRENT TOO HIGH, requested " + str(currentCommand) + " mA")
            self.current_cmd = CURRENT_LIMIT
            # setMotorCurrent(self.dev_id, self.side*CURRENT_LIMIT);  # set the current on the exo
            fxs.send_motor_command(self.dev_id, fxe.FX_CURRENT, CURRENT_LIMIT)

    """
    # initialize the torque profile
    """
    def init_collins_profile(self, mass=None, ramp_start_percent_gait=None, onset_percent_gait=None, peak_percent_gait=None,
                             stop_percent_gait=None, onset_torque=None, normalized_peak_torque=None):
        # average values from the zhang/collins optimization paper.
        # t0 = 0;
        # t1 = 27.1;
        # t2 = 50.4;
        # t3 = 62.7;
        # ts = 2;

        # peakTorqueNormalized = 0.20; # 0.76; # Using a smaller value due to Dephy Exo Limit.

        if (mass != None):
            self.user_mass = mass  # kg
        if (ramp_start_percent_gait != None):
            self.t0 = ramp_start_percent_gait
        if (onset_percent_gait != None):
            self.t1 = onset_percent_gait
        if (peak_percent_gait != None):
            self.t2 = peak_percent_gait
        if (stop_percent_gait != None):
            self.t3 = stop_percent_gait
        if (onset_torque != None):
            self.ts = onset_torque
        if (normalized_peak_torque != None):
            self.peak_torque_normalized = normalized_peak_torque

        if (self.user_mass != -1 and self.t0 != -1,
            self.t1 != -1 and self.t2 != -1 and self.t3 != -1 and self.ts != -1 and self.peak_torque_normalized != -1):

            self.tp = self.user_mass * self.peak_torque_normalized;

            self.a1 = (2 * (self.tp - self.ts)) / m.pow((self.t1 - self.t2), 3);
            self.b1 = -((3 * (self.t1 + self.t2) * (self.tp - self.ts)) / m.pow((self.t1 - self.t2), 3));
            self.c1 = (6 * self.t1 * self.t2 * (self.tp - self.ts)) / m.pow((self.t1 - self.t2), 3);
            self.d1 = -((-m.pow(self.t1, 3) * self.tp + 3 * m.pow(self.t1, 2) * self.t2 * self.tp - 3 * self.t1 * m.pow(
                self.t2, 2) * self.ts + m.pow(self.t2, 3) * self.ts) / m.pow((self.t1 - self.t2), 3));

            self.a2 = -((self.tp - self.ts) / (2 * m.pow((self.t2 - self.t3), 3)));
            self.b2 = (3 * self.t3 * (self.tp - self.ts)) / (2 * m.pow((self.t2 - self.t3), 3));
            self.c2 = (3 * (m.pow(self.t2, 2) - 2 * self.t2 * self.t3) * (self.tp - self.ts)) / (
                        2 * m.pow((self.t2 - self.t3), 3));
            self.d2 = -((3 * m.pow(self.t2, 2) * self.t3 * self.tp - 6 * self.t2 * m.pow(self.t3, 2) * self.tp + 2 * m.pow(
                self.t3, 3) * self.tp - 2 * m.pow(self.t2, 3) * self.ts + 3 * m.pow(self.t2, 2) * self.t3 * self.ts) / (
                                    2 * m.pow((self.t2 - self.t3), 3)));

        else:
            print('ExoBoot :: init_collins_profile : one of the parameters is not set' + \
                  '\n user_mass : ' + str(self.user_mass) + \
                  '\n ramp_start_percent_gait : ' + str(self.t0) + \
                  '\n onset_percent_gait : ' + str(self.t1) + \
                  '\n peak_percent_gait : ' + str(self.t2) + \
                  '\n stop_percent_gait : ' + str(self.t3) + \
                  '\n onset_torque : ' + str(self.ts) + \
                  '\n normalized_peak_torque : ' + str(self.peak_torque_normalized))

    """
    # start to run the torque profile
    """
    # def run_collins_profile(self, external_read=False):
    def run_collins_profile(self, sensor_data):
        # update data
        # if not external_read:
        self.read_data(sensor_data)
        # print('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ ' : percent_gait : ' + str(self.percent_gait))

        if self.percent_gait != -1:
            if (self.percent_gait <= self.t1) and (self.t0 <= self.percent_gait):  # torque ramp to ts at t1
                # 1 cout << "exoBoot :: runCollinsProfile : In t1 region" << endl;
                if self.mode != fxe.FX_CURRENT:
                    self.set_controller(fxe.FX_CURRENT)

                tau = self.ts / (self.t1 - self.t0) * self.percent_gait - self.ts / (self.t1 - self.t0) * self.t0;
                # 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
                self.set_exo_current(max(NO_SLACK_CURRENT, A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau)))))  # Commented out till output tested.
            # print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T1 Region : tau : ' + str(tau) )

            elif self.percent_gait <= self.t2:  # the rising spline
                # 1 cout << "exoBoot :: runCollinsProfile : In t2 region" << endl;
                if self.mode != fxe.FX_CURRENT:
                    self.set_controller(fxe.FX_CURRENT)

                tau = self.a1 * m.pow(self.percent_gait, 3) + self.b1 * m.pow(self.percent_gait,2) + self.c1 * self.percent_gait + self.d1;
                # 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
                self.set_exo_current(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau))))  # Commented out till output tested.
                # print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T2 Region : tau : ' + str(tau) )

            elif self.percent_gait <= self.t3:  # the falling spline
                # 1 cout << "exoBoot :: runCollinsProfile : In t3 region" << endl;
                if self.mode != fxe.FX_CURRENT:
                    set_controller(fxe.FX_CURRENT)

                tau = self.a2 * m.pow(self.percent_gait, 3) + self.b2 * m.pow(self.percent_gait,2) + self.c2 * self.percent_gait + self.d2;
                # 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
                self.set_exo_current(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau))))  # Commented out till output tested.
                # print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T3 Region : tau : ' + str(tau) )

            else:  # go to the slack position if we aren't commanding a specific value
                tau = 0
                self.set_exo_current(NO_SLACK_CURRENT) # just enough to keep a small tension in the cable

    def percent_gait_calc(self):
        if -1 != self.expected_duration:  # if the expected duration is set calculate the percent gait
            self.percent_gait = 100 * (self.data_current[self.idx_time] - self.heelstrike_timestamp_current) / self.expected_duration

        if 100 < self.percent_gait:  # if it has gone past 100 just hold 100
            self.percent_gait = 100


    # print ('exoBoot :: percent_gait_calc : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  percent_gait : ' + str(self.percent_gait) )

    def update_expected_duration(self):
        # TODO : In addition to checking that the step time is within a range, also check the the time it is armed is within the typical time.  Common errors occur from short spikes in acceleration that can have a close frequency.

        step_time = self.heelstrike_timestamp_current - self.heelstrike_timestamp_previous
        # armed_time = 0
        # if self.armed_timestamp != -1 :
        # armed_time = self.heelstrike_timestamp_current - self.armed_timestamp
        if -1 == self.heelstrike_timestamp_previous:  # if it is the first time running just record the timestamp
            self.heelstrike_timestamp_previous = self.heelstrike_timestamp_current
            return
        if -1 in self.past_gait_times:  # if all the values haven't been replaced
            self.past_gait_times.insert(0, step_time)  # insert the new value at the beginning
            self.past_gait_times.pop()  # remove the last value
        elif (step_time <= 1.75 * max(self.past_gait_times)) and (step_time >= 0.25 * min(self.past_gait_times)):  # and (armed_time > ARMED_DURATION_PERCENT * self.expected_duration)): # a better check can be used.  If the person hasn't stopped or the step is good update the vector.
            # !!!THE ARMED TIME CHECK STILL NEEDS TO BE TESTED!!!
            self.past_gait_times.insert(0, step_time)  # insert the new value at the beginning
            self.past_gait_times.pop()  # remove the last value
            # TODO: Add rate limiter for change in expected duration so it can't make big jumps
            self.expected_duration = sum(self.past_gait_times) / len(self.past_gait_times);  # Average to the nearest ms


    # print ('exoBoot :: update_expected_duration : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  expected_duration : ' + str(self.expected_duration) )

    def clear_gait_estimate(self):
        self.past_gait_times = [-1] * NUM_GAIT_TIMES_TO_AVERAGE  # store the most recent gait times
        self.expected_duration = -1  # current estimated gait duration


    def check_for_heelstrike(self):
        # the trigger on the inversion of the leg is one method.
        # can also use spikes in acceleration (X seems to be best candidate but may not work well for slower gaits with smaller impacts)
        # Other candidates also possible.
        triggered = False
        armed_time = 0

        if self.armed_timestamp != -1:
            armed_time = self.data_current[self.idx_time] - self.armed_timestamp

        # due to the IMUs that mount on both boots
        # for right boot, the segmentation_arm_threashold needs to be negative
        # for left boot, the segmentation_arm_threashold needs to be positive
        if (self.idx == 1):
            self.segmentation_arm_threashold = -1*self.segmentation_arm_threashold_raw
        # trigger the heel strike event
            if (not self.heelstrike_armed) and (self.data_current[self.idx_gyro_z] <= self.segmentation_arm_threashold) and (self.data_current[self.idx_gyro_x] >= 50*gyro_conversion_factor):
                # if ((not self.heelstrike_armed) and self.data_current[self.idx_gyro_z] >= self.segmentation_arm_threashold) :
                self.heelstrike_armed = True
                self.armed_timestamp = self.data_current[self.idx_time]

            # reset and be prepared for the heel strike event
            if (self.heelstrike_armed and (self.data_current[self.idx_gyro_z] >= self.segmentation_trigger_threashold) and (
                    self.data_current[self.idx_gyro_x] <= 0)):
                # if (self.heelstrike_armed and (self.data_current[self.idx_gyro_z] <= self.segmentation_trigger_threashold) ) :
                self.heelstrike_armed = False
                self.armed_timestamp = -1
                if armed_time > ARMED_DURATION_PERCENT / 100 * self.expected_duration:
                    triggered = True

        else:
            self.segmentation_arm_threashold = self.segmentation_arm_threashold_raw
        # trigger the heel strike event
            if (not self.heelstrike_armed) and (self.data_current[self.idx_gyro_z] >= self.segmentation_arm_threashold) and (self.data_current[self.idx_gyro_x] >= 50*gyro_conversion_factor):
                # if ((not self.heelstrike_armed) and self.data_current[self.idx_gyro_z] >= self.segmentation_arm_threashold) :
                self.heelstrike_armed = True
                self.armed_timestamp = self.data_current[self.idx_time]

            # reset and be prepared for the heel strike event
            if (self.heelstrike_armed and (self.data_current[self.idx_gyro_z] <= self.segmentation_trigger_threashold) and (
                    self.data_current[self.idx_gyro_x] <= 0)):
                # if (self.heelstrike_armed and (self.data_current[self.idx_gyro_z] <= self.segmentation_trigger_threashold) ) :
                self.heelstrike_armed = False
                self.armed_timestamp = -1
                if armed_time > ARMED_DURATION_PERCENT / 100 * self.expected_duration:
                    triggered = True

        self.segmentation_trigger = triggered


    # print ('exoBoot :: check_for_heelstrike : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ ' : GyroZ : ' + str(self.data_current[self.idx_gyro_z] ) + ' : heelstrike_armed : ' + str(self.heelstrike_armed) + ' : triggered : ' + ('True' if triggered else 'False'))

    def __del__(self):
        # TODO: make where it can take single or multiple values
        # if self.should_log :
        if self.data_file is not None:
            self.data_file.close()
        # need to change api here
        fxs.stop_streaming(self.dev_id)
        # fxStopStreaming(self.dev_id)
        fxs.close(self.dev_id)

"""
# Adding soft sensor readings via arduino
"""
class Arduino:

    def __init__(self):
        self.rawdata1 = -1
        self.rawdata2 = -1
        self.rawdata3 = -1
        self.rawdata4 = -1
        print('connecting to arduino')
        self.arduino = self._connectToArduino()  # connect to the ardunio
        # self.obtain_softsensor_data()  # read in the current data from the system

    def _connectToArduino(self):
        arduino_ports = [
            p.device
            for p in serial.tools.list_ports.comports()
            if 'Arduino' in p.description
        ]
        if not arduino_ports:
            raise IOError("No Arduino found")
        if len(arduino_ports) > 1:
            warnings.warn('Multiple Arduinos found - using the first')

        # Arduino = serial.Serial(arduino_ports[0], baudrate=57600)
        arduino = serial.Serial(arduino_ports[0], baudrate=2000000)
        arduino.flush()
        # Arduino.reset_input_buffer()
        return arduino

    def obtain_softsensor_data(self):
        soft_sensor = self.arduino
        while (soft_sensor.inWaiting() == 0):
            pass
        try:
            data = soft_sensor.readline()
            dataarray = data.decode().rstrip().split("\t")
            # arduino.reset_input_buffer()
            # print(dataarray)
            self.rawdata1 = float(dataarray[0])  # raw value of the textile sensor 1
            self.rawdata2 = float(dataarray[1])  # raw value of the textile sensor 2
            self.rawdata3 = float(dataarray[2])  # raw value of the textile sensor 3
            self.rawdata4 = float(dataarray[3])  # raw value of the textile sensor 4
            # return rawdata1, rawdata2, rawdata3, rawdata4
        except (KeyboardInterrupt, SystemExit, IndexError, ValueError):
            pass