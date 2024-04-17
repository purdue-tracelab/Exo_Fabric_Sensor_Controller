import os
import sys
import platform

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from flexsea import flexsea as flex
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe


from collections import deque
import math as m
from datetime import datetime
import configparser
fxs = flex.FlexSEA()
# set current limit to 1A for baseline test
CURRENT_LIMIT = 1000 # mA : Current limit for the motor command.  You should still set the values in boots protection circuit because the PI controller and position controller can still push it higher.

QUE_LENGTH = 50 # number of samples to use for the LSTM inferance.
# No need for left and right signs for new api?
# LEFT = -1	# modifies values for the left boot
# RIGHT = 1	# modifies values for the right boot

# this was used with c since python handles types itself we will just make this one.
FIXED_POINT_COEFF_DEG = 1
def FIXED_POINT_CONVERSION_DEG(angle)	:
	return (angle)/FIXED_POINT_COEFF_DEG

# this was used with c since python handles types itself we will just make this one.
FIXED_POINT_COEFF_MM = 1
def FIXED_POINT_CONVERSION_MM(distance) :
	return (distance) / FIXED_POINT_COEFF_MM

# this was used with c since python handles types itself we will just make this one.
FIXED_POINT_COEFF_ACCL = 1
def FIXED_POINT_CONVERSION_ACCL(acceleration) :
	return (acceleration) / FIXED_POINT_COEFF_ACCL

# this was used with c since python handles types itself we will just make this one.
FIXED_POINT_COEFF_GYRO  = 1
def FIXED_POINT_CONVERSION_GYRO(angularVelocity) :
	return (angularVelocity) / FIXED_POINT_COEFF_GYRO

TICKS_TO_ANGLE_COEFF =  0.02197 * FIXED_POINT_COEFF_DEG			# converts encoder ticks to degrees
ANGLE_TO_TICKS_COEFF = 1/TICKS_TO_ANGLE_COEFF					# converts angles to moter ticks
BIT_TO_ACCL_COEFF	= 1 / 8192 * 9.8 * FIXED_POINT_COEFF_ACCL	# converts accelerometer readings to acceleration m/s^2
BIT_TO_GYRO_COEFF	= 1 / 32.8 * FIXED_POINT_COEFF_GYRO			# converts gyro reading to angular velocity deg/s


SLACK_STEP_SIZE = (3 * ANGLE_TO_TICKS_COEFF)	# this limits how fast the slack position can change.

MAX_FILE_SIZE = 500 * 10**6 # 500 MB : limits the file size writting so you don't have to open a really big file

NUM_GAIT_TIMES_TO_AVERAGE = 4	# for the gait duration estimate how many gait cycles to average.  More changes slower but is more stable.  This might become obsolete if we move to an infinate horizon filter.

ARMED_DURATION_PERCENT = 10
# set no slack current to be 200 for now
#NO_SLACK_CURRENT = 400
NO_SLACK_CURRENT = 200
# convert degrees to radians
def DEG_TO_RAD(angle_deg)  :
	return (angle_deg) * m.pi / 180

# convert mNm or Nmm to Nm
def NMM_TO_NM(torque)  :
	return (torque) / 1000

# convert Nm to mNm or Nmm
def NM_TO_NMM(torque)  :
	return (torque) * 1000

# convert A to mA
def A_TO_MA(torque)  :
	return (torque) * 1000

# convert mA to A
def MA_TO_A(torque)  :
	return (torque) / 1000


#
# Class to interface with the boot
#
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
	#	shouldExoLog: Should the boots program log data independently
	#	shouldAuto	: Should the boot automatically add data to the buffer (true), if false it will only do that when requested.
	#	sync_pin	: Which pin is use for the sync led
	#	mode		: What pin refrence is used "BOARD" is more compatable between the RPi and other boards
	#	use_tactor	: Should a tactor be used.  This is convenient to make false if you just want to test the boots without the peripheral board.
	#
	def __init__ (self, devSide, port, baud_rate, idx, frequency = 1000,  shouldLog = True, shouldAuto = 1):
		# Attach the values to the instance.
		print('boot constructor : top')
		self.port = port
		self.baud_rate = baud_rate
		self.idx = idx
		# self.side = devSide #no longer needed for new boots

		self.frequency = frequency
		self.should_log = shouldLog
		self.should_auto = shouldAuto

		# Values used for zeroing the encoder position
		self.motor_ticks_offset=0
		self.ankle_ticks_offset=0


		# Zhang/Collins parameters
		self.t0 = -1		# ramp start percent
		self.t1 = -1		# ramp end percent
		self.t2 = -1		# peak percent
		self.t3 = -1		# drop percent
		self.ts = -1		# top torque for the ramp
		self.tp = -1		# peak torque
		self.user_mass = -1	# user mass
		self.peak_torque_normalized = -1	# torque normalized to user mass

		# parameters for the Zhang/Collins torque curve
		self.a1 = -1
		self.b1 = -1
		self.c1 = -1
		self.d1 = -1

		self.a2 = -1
		self.b2 = -1
		self.c2 = -1
		self.d2 = -1


		self.segmentation_trigger = False	# goes high when heelstrike detected
		self.heelstrike_armed = False		# high when trigger armed
# may need to change the threshold that determine the heelstrike		
        self.segmentation_arm_threashold = 150 * FIXED_POINT_COEFF_GYRO	# the threashold that must be gone above to arm the trigger
		self.segmentation_trigger_threashold = 0	# the theashold that must be dropped below to trigger the heelstrike

		self.past_gait_times = [-1] * NUM_GAIT_TIMES_TO_AVERAGE	# store the most recent gait times

		self.expected_duration = -1	# current estimated gait duration

		self.heelstrike_timestamp_current = -1	# Timestamp of the most recent heelstrike
		self.heelstrike_timestamp_previous = -1	# Timestamp of the second most recent heelstrike
		self.armed_timestamp = -1		# timestamp of wsegmentation_trigger_threasholdhen the trigger was armed
		self.percent_gait = -1			# estimate of the percent of gait
		self.percent_gait_previous = -1	# the previous loops estimate of the percent gait. used to determine crossing of timepoints

		self.current_cmd = None	# how much current are we requesting

		# get the labels of the values we are streaming from the boot
		self.labels_stream = ["State time", 	\
			"Accel X", 	"Accel Y", 	"Accel Z", 	\
			"Gyro X", 	"Gyro Y",	"Gyro Z", 	\
			"Encoder angle", "Encoder velocity",   "Encoder acceleration",	\
			"Motor voltage", "Motor current",	\
			"Battery voltage", "Battery current", \
			"Ankle angle", "Ankle vel"
		]

# Change name of the stream variables to match the actpack variables of new api
# Check felxsea/dev_spec/ActPackState.py, i.e., act_pack.mot_cur
		# store the appropriate numbers for reading those values from the boot, check fxUtil.py for details
		self.vars_to_stream = [ FX_STATETIME, 		\
				FX_ACCELX,	FX_ACCELY,	FX_ACCELZ, 	\
				FX_GYROX,  	FX_GYROY,  	FX_GYROZ,	\
				FX_ENC_ANG,	FX_ENC_VEL,	FX_ENC_ACC,	\
				FX_MOT_VOLT, FX_MOT_CURR,	\
				FX_BATT_VOLT, FX_BATT_CURR, \
				FX_ANKLE_ANG, FX_ANKLE_ANG_VEL
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
		self.idx_other_base = 16
		# Control system info
		#self.idx_sync = self.idx_ankle_vel + 1
		self.idx_percent_gait = self.idx_other_base
		self.idx_heelstrike_armed = self.idx_other_base + 1
		self.idx_segmentation_trigger = self.idx_other_base + 2
		self.idx_expected_duration = self.idx_other_base + 3
		self.idx_current_cmd = self.idx_other_base + 4
		self.idx_torque_cmd = self.idx_other_base + 5
        
# need to change api here, fxSetStreamVariables and fxStartStreaming are not the new api
		print('boot constructor : connecting to device')
		self.dev_id = self._connectToDevice()	# connect to the boot
		print('boot constructor : device connected : id : ' , self.dev_id)
		fxSetStreamVariables(self.dev_id,self.vars_to_stream)	# set the variables to stream
		if not fxStartStreaming(self.dev_id,self.frequency,self.should_log,self.should_auto): # start streaming
			raise Exception('Streaming failed')
		else:
			sleep(0.4)

		# the labels of the other values we are writing that don't come from the boot
		self.labels_other = [
			"Percent Gait", "Heelstrike Armed", "Segmentation Trigger", "Expected Duration", "Current Command", "Torque Command"]

		# create a deque that the LSTM will use for its inference.
		self.data_que = deque(maxlen = QUE_LENGTH)
		self.data_exo = [-1] * len(self.labels_stream)	# fill in the data with -1 till we read in the values
		self.data_other = [-1] * len(self.labels_other) # fill in the data with -1 till we read in the values
		self.labels_current = self.labels_stream + self.labels_other	# combine the labels

		self.data_current = self.data_exo + self.data_other	# combine the data

		self.data_file = None	# store the data file handle
		if self.should_log :	# if we are logging create the file
			self.file_base = "";
			self.file_extension = "";
			self.data_filename = "";

			# initilize the logging
			self.data_file = self.log_init()

		self.read_data()	# read in the current data from the system

		self.data_current = [0] * len(self.labels_stream) # clear this variable since the first read will not have all the zeroing set
		self.data_que.clear() # clear this variable since the first read will not have all the zeroing set

		self.kt = 140 		# motor torque constant Nmm/A 140 is the torque constant given by Dephy.

		# controller properties
		self.currentKp = 0
		self.currentKi = 0
        self.currentff = 0

	#
	# this is pulled from the Dephy examples
	#
	def _connectToDevice(self):
		fxs.open(self.port, self.baud_rate, 0)
		timeElapsed = 0
		TIMEOUT_LIMIT = 10
#need to change here, fxIsOpen is not the new api
		while(timeElapsed <= TIMEOUT_LIMIT and not fxIsOpen(self.idx)):
			# There is certainly a better way to do this
			sleep(0.2)
			timeElapsed += 0.2

		if(not fxIsOpen(self.idx)):
			raise Exception("Couldn't connect to port {}".format(self.port))

		sleep(0.1)
		MAX_DEVICE_ID_ATTEMPTS = 10
		num_attempts = 0
#need to change here, fxGetDeviceIds() is not the new api        
		devIds = fxGetDeviceIds()
		while(num_attempts < MAX_DEVICE_ID_ATTEMPTS and (len(devIds) == 0 or len(devIds) < (self.idx+1))):
		# added in a check because it was only recognizing the first idx when running it in a script.  When manually running it would error the first time but be ok the second time.
			sleep(0.2)
			devIds = fxGetDeviceIds()
			print ("attempt number : " + str(num_attempts))
		print("devIds : ")
		print (devIds)
		if len(devIds) == 0:
			raise Exception('Failed to get device Id')
		dev_id = devIds[self.idx]
		print("Devid is: ", dev_id)
		return dev_id

	#
	# rotate two imu data to be consistent
	#
	def rotate_imu (self, accl_x_raw, accl_y_raw, accl_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw):
		angleOffset = 0
		#angleOffset = m.pi
		rotAngleZ = 0
		rotAngleX = 0

		# first rotate about the Z axis to get the x axis forward on the boot
		# then rotate about the new x axis to get the y axis pointing up

		if (self.side == LEFT) :
			rotAngleZ = 0
			rotAngleX = angleOffset

		else :
			rotAngleZ = 0
			rotAngleX = 0

		acclX = 	BIT_TO_ACCL_COEFF * (accl_x_raw* m.cos(rotAngleZ) + \
						accl_y_raw * -m.sin(rotAngleZ) )
		acclY = 	BIT_TO_ACCL_COEFF * (accl_x_raw * m.sin(rotAngleZ) * m.cos(rotAngleX) + \
						accl_y_raw * m.cos(rotAngleZ) * m.cos(rotAngleX) + \
						accl_z_raw * -m.sin(rotAngleX))
		acclZ = 	BIT_TO_ACCL_COEFF * (accl_x_raw * m.sin(rotAngleZ) * m.sin(rotAngleX) + \
						accl_y_raw * m.cos(rotAngleZ) * m.sin(rotAngleX) + \
						accl_z_raw * m.cos(rotAngleX))

		gyroX = 	BIT_TO_GYRO_COEFF * (gyro_x_raw* m.cos(rotAngleZ) + \
						gyro_y_raw * -m.sin(rotAngleZ))
		gyroY = 	BIT_TO_GYRO_COEFF * (gyro_x_raw * m.sin(rotAngleZ) * m.cos(rotAngleX) + \
						gyro_y_raw * m.cos(rotAngleZ) * m.cos(rotAngleX) + \
						gyro_z_raw * -m.sin(rotAngleX))
		gyroZ = 	BIT_TO_GYRO_COEFF * (gyro_x_raw * m.sin(rotAngleZ) * m.sin(rotAngleX) + \
						gyro_y_raw * m.cos(rotAngleZ) * m.sin(rotAngleX) + \
						gyro_z_raw * m.cos(rotAngleX))

		return [acclX, acclY, acclZ, gyroX, gyroY, gyroZ]


# this zero encoder may not be used for new api?
	#
	#	Record the motor and ankle position when the function is called
	#
	def zero_encoders(self):
		self.read_data();
		self.motor_ticks_offset = self.motorTicksRaw;
		self.ankle_ticks_offset = self.ankleTicksRaw;


	#
	#	Update the current sensor information from the system
	#
	def read_data(self):
    # need to deal w self.vars_to_stream, to log the data, this may not be the best way for the new api
		# self.data_exo =  fxReadDevice(self.dev_id,self.vars_to_stream)
        self.data_exo =  fxs.read_device(self.dev_id,self.vars_to_stream)
		if (self.data_current[self.idx_time]  != self.data_exo[self.idx_time] ) : # update the data if new data has come in

			# !!! Make sure signs are correct.
			self.motorTicksRaw = self.data_exo[self.idx_motor_angle]
			self.motorTicksZeroed =  self.side * (self.motorTicksRaw - self.motor_ticks_offset);  # remove the offset, and adjust for side

			self.ankleTicksRaw = self.data_exo[self.idx_ankle_angle]
			self.ankleTicksZeroed = self.side * (self.ankleTicksRaw - self.ankle_ticks_offset) # remove the offset, and adjust for side

			accl_gyro_rotated = self.rotate_imu(self.data_exo[self.idx_accl_x], self.data_exo[self.idx_accl_y], self.data_exo[self.idx_accl_z], self.data_exo[self.idx_gyro_x], self.data_exo[self.idx_gyro_y], self.data_exo[self.idx_gyro_z])

			self.data_current = [-1] * len(self.data_exo)  # clear it.  I will be extending the data_other to the end and I don't want issues.

			self.data_current[self.idx_time] = self.data_exo[self.idx_time]
			self.data_current[self.idx_accl_x] = accl_gyro_rotated[0]
			self.data_current[self.idx_accl_y] = accl_gyro_rotated[1]
			self.data_current[self.idx_accl_z] = accl_gyro_rotated[2]
			self.data_current[self.idx_gyro_x] = accl_gyro_rotated[3]
			self.data_current[self.idx_gyro_y] = accl_gyro_rotated[4]
			self.data_current[self.idx_gyro_z] = accl_gyro_rotated[5]
			self.data_current[self.idx_motor_angle] = self.ticks_to_angle(self.motorTicksZeroed)
			self.data_current[self.idx_motor_vel] = self.data_exo[self.idx_motor_vel]
			self.data_current[self.idx_motor_accl] = self.data_exo[self.idx_motor_accl]
			self.data_current[self.idx_motor_voltage] = self.data_exo[self.idx_motor_voltage]
			self.data_current[self.idx_motor_current] = self.data_exo[self.idx_motor_current]
			self.data_current[self.idx_batt_voltage] = self.data_exo[self.idx_batt_voltage]
			self.data_current[self.idx_batt_current] = self.data_exo[self.idx_batt_current]
			self.data_current[self.idx_ankle_angle] = self.ticks_to_angle(self.ankleTicksZeroed)
			self.data_current[self.idx_ankle_vel] = self.data_exo[self.idx_ankle_vel]

			self.data_other =[-1] * len(self.labels_other)

			self.check_for_heelstrike()

			if (self.segmentation_trigger) :
				self.heelstrike_timestamp_previous = self.heelstrike_timestamp_current
				self.heelstrike_timestamp_current = self.data_current[self.idx_time]
				self.update_expected_duration()

			self.percent_gait_previous = self.percent_gait
			self.percent_gait_calc()

			self.torque_cmd = (self.current_cmd/1000 * self.kt if self.current_cmd != None else None)

			# Gait Estimation Data
			self.data_other[self.idx_percent_gait - self.idx_other_base] = self.percent_gait
			self.data_other[self.idx_heelstrike_armed - self.idx_other_base] = (1 if self.heelstrike_armed else 0)
			self.data_other[self.idx_segmentation_trigger - self.idx_other_base] = (1 if self.segmentation_trigger else 0)
			self.data_other[self.idx_expected_duration - self.idx_other_base] = self.expected_duration
			self.data_other[self.idx_current_cmd - self.idx_other_base] = self.current_cmd
			self.data_other[self.idx_torque_cmd - self.idx_other_base] = self.torque_cmd

			self.data_current.extend(self.data_other)

			if self.should_log :
				self.log()

			self.data_que.append(self.data_current)




	# def defineGains (void); 					# default values all to 0
	# def defineGains (int, int, int);			# takes controller type and sets Kp and Ki for that controller
	# def defineGains (int, int, int, int, int);	# as above but also takes in the stiffness and damping for the impedance controller
	# def defineKB (int, int); # for the impedance mode set the stiffness and damping without touching the gains
	# def displayControllerState (void); # displays the current state
	# def exoControllerInit(void);


	def log_init(self) :

		start_time = datetime.now()
		time_str = start_time.strftime("%Y_%m_%d_%Hh%Mm")
		side_str = "_left" if self.side == LEFT else "_right"

		# you need to initilize these before calling these
		self.file_base = time_str + side_str
		self.file_extension = ".csv"

		self.data_filename = self.get_free_filename(self.file_base, self.file_extension)
		# cut start
		# self.data_filename = self.file_base + self.file_extension
		# print("filename : " + self.data_filename )
		# i = 0
		# while os.path.exists(self.data_filename):
			# i +=1
			# self.data_filename =  self.file_base + "_"+ str(i) + self.file_extension
		#  cut end
		data_file = open(self.data_filename, 'a')

		labels_csv = ""
		for label in self.labels_current :
			labels_csv += label + ", "
		data_file.write(labels_csv)
		data_file.write("\n ")

		return data_file


	def get_free_filename(self, base, extension) :
		data_filename = base + extension
		i = 0
		while os.path.exists(data_filename):
			i +=1
			data_filename =  base + "_"+ str(i) + extension
		return data_filename


	def log (self) :
		if self.should_log :
			# check file size
			if os.path.getsize(self.data_filename) > MAX_FILE_SIZE :
				self.data_file.close()
				self.data_filename = self.get_free_filename(self.file_base, self.file_extension)
				self.data_file = open(self.data_filename, 'a')

				labels_csv = "";
				for label in self.labels_current :
					labels_csv += label + ", "
				self.data_file.write(labels_csv)
				self.data_file.write("\n ")

			for i in range(0, len(self.data_current)):
				self.data_file.write(str(self.data_current[i]) if self.data_current[i] != None else 'nan')  # write out the data.  If the type is None change to 'nan' so it is easy for matlab to parse.
				self.data_file.write(",")
			self.data_file.write("\n ")
		else :
			print("exo_defs :: log(self) : \n\tYOU WANTED TO LOG VALUES BUT LOGGING DATA WAS NOT SELECTED")


	def ankle_torque_to_current(self, torque):

		# get the current based on the torque cmd and the system state
		current = torque / self.kt;

		return current;
# ankle angle conversion in here is not needed.
	def ticks_to_angle(self, ticks) :
		return ticks * TICKS_TO_ANGLE_COEFF

	def define_current_gains (self, kp, ki, ff) : # for impedance also take in the stiffness and damping
		self.currentKp	= kp;
		self.currentKi	= ki;
        self.currentff  = ff;
# check the api that modified in here
	def set_controller(self, controlMode) :
		self.mode = controlMode;	# store the mode so we can easily check what is set as the Dephy lib doesn't return anything to know the current mode
		fxs.send_motor_command(dev_id, fxe.FX_NONE, 0);  # change the control mode to the one requested,  THIS MUST BE DONE BEFORE THE GAINS ARE SENT TO THE EXO

		if self.mode == fxe.FX_NONE :
			self.current_cmd = None
			set_gains(self.dev_id, 0, 0, 0, 0, 0, 0);  # clear the gains
            fxs.send_motor_command(dev_id, fxe.FX_NONE, 0);
            
		elif self.mode == fxe.FX_CURRENT:
				self.current_cmd = 0
				self.set_exo_current(0);  # set the current to zero to avoid surprises
				set_gains(self.dev_id, self.currentKp, self.currentKi, 0, 0, 0, self.currentff);  # set the gains for the current control

		else :
			self.current_cmd = None
			set_gains(self.dev_id, 0, 0, 0, 0, 0, 0);  # clear the gains
            fxs.send_motor_command(dev_id, fxe.FX_NONE, 0);
	#
	#	Sends the current command accounting for side, positive is plantar flexion
	#
	# def set_exo_current(self, currentCommand) :
		# if (abs(currentCommand) < CURRENT_LIMIT) :
			# self.current_cmd = currentCommand
			# setMotorCurrent(self.dev_id, self.side*currentCommand);  # set the current on the exo

		# else :
			# print("exoBoot :: set_exo_current : CURRENT TOO HIGH, requested " + str(currentCommand) + " mA")
			# self.current_cmd = CURRENT_LIMIT
			# setMotorCurrent(self.dev_id, self.side*CURRENT_LIMIT);  # set the current on the exo
			# # cout << "exoBoot :: set_exo_current : CURRENT TOO HIGH, requested " << currentCommand << " mA" << endl;
# check the api that modified in here
	def set_exo_current(self, currentCommand) :
		if (abs(currentCommand) < CURRENT_LIMIT) :
			self.current_cmd = currentCommand
			# setMotorCurrent(self.dev_id, self.side*currentCommand);  # set the current on the exo
            fxs.send_motor_command(dev_id, fxe.FX_CURRENT, currentCommand);
		else :
			print("exoBoot :: set_exo_current : CURRENT TOO HIGH, requested " + str(currentCommand) + " mA")
			self.current_cmd = CURRENT_LIMIT
			#setMotorCurrent(self.dev_id, self.side*CURRENT_LIMIT);  # set the current on the exo
            fxs.send_motor_command(dev_id, fxe.FX_CURRENT, CURRENT_LIMIT);


	def init_collins_profile(self, mass = None, ramp_start_percent_gait = None, onset_percent_gait = None, peak_percent_gait = None, stop_percent_gait = None,  onset_torque = None, normalized_peak_torque = None) :
		# average values from the zhang/collins optimization paper.
		# t0 = 0;
		# t1 = 27.1;
		# t2 = 50.4;
		# t3 = 62.7;
		# ts = 2;

		# peakTorqueNormalized = 0.20; # 0.76; # Using a smaller value due to Dephy Exo Limit.

		if (mass != None) :
			self.user_mass = mass # kg
		if (ramp_start_percent_gait != None) :
			self.t0 = ramp_start_percent_gait
		if (onset_percent_gait != None) :
			self.t1 = onset_percent_gait
		if (peak_percent_gait != None) :
			self.t2 = peak_percent_gait
		if (stop_percent_gait != None) :
			self.t3 = stop_percent_gait
		if (onset_torque != None) :
			self.ts = onset_torque
		if (normalized_peak_torque != None) :
			self.peak_torque_normalized = normalized_peak_torque


		if (self.user_mass != -1 and self.t0  != -1, self.t1  != -1 and self.t2  != -1 and self.t3  != -1 and self.ts  != -1and self.peak_torque_normalized  != -1) :

			self.tp = self.user_mass * self.peak_torque_normalized;

			self.a1 = (2 *(self.tp - self.ts))/m.pow((self.t1 - self.t2),3);
			self.b1 = -((3 *(self.t1 + self.t2) *(self.tp - self.ts)) / m.pow((self.t1 - self.t2),3));
			self.c1 = (6* self.t1 * self.t2 * (self.tp - self.ts))/ m.pow((self.t1 - self.t2),3);
			self.d1 = -((-m.pow(self.t1, 3) *self.tp + 3 * m.pow(self.t1, 2)* self.t2 * self.tp - 3 * self.t1 * m.pow(self.t2,2) * self.ts + m.pow(self.t2,3) * self.ts)/ m.pow((self.t1 - self.t2),3));


			self.a2 = -((self.tp - self.ts)/(2* m.pow((self.t2 - self.t3),3)));
			self.b2 = (3 *self.t3 *(self.tp - self.ts))/(2 * m.pow((self.t2 - self.t3),3));
			self.c2 = (3 *(m.pow(self.t2,2) - 2 *self.t2 *self.t3) * (self.tp - self.ts))/(2* m.pow((self.t2 - self.t3),3));
			self.d2 = -((3 * m.pow(self.t2,2) * self.t3 * self.tp - 6 * self.t2 * m.pow(self.t3, 2) * self.tp + 2 * m.pow(self.t3,3) * self.tp - 2 * m.pow(self.t2,3) * self.ts + 3 * m.pow(self.t2, 2) * self.t3 * self.ts)/(2 * m.pow((self.t2 - self.t3), 3)));

		else :
			print('ExoBoot :: init_collins_profile : one of the parameters is not set' + \
				'\n user_mass : ' + str(self.user_mass) + \
				'\n ramp_start_percent_gait : ' + str(self.t0) + \
				'\n onset_percent_gait : ' + str(self.t1) + \
				'\n peak_percent_gait : ' + str (self.t2) + \
				'\n stop_percent_gait : ' + str (self.t3) + \
				'\n onset_torque : ' + str (self.ts) + \
				'\n normalized_peak_torque : ' + str (self.peak_torque_normalized))

# need to change the ctrl_current api here. 
	def run_collins_profile(self, external_read = False) :
		# update data
		if not external_read :
			self.read_data()
		#print('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ ' : percent_gait : ' + str(self.percent_gait))

		if (self.percent_gait != -1) :
			if ((self.percent_gait <= self.t1)  and  (self.t0 <= self.percent_gait)) : # torque ramp to ts at t1
				# 1 cout << "exoBoot :: runCollinsProfile : In t1 region" << endl;
				if (self.mode != CTRL_CURRENT) :
					self.set_controller (CTRL_CURRENT);

				tau = self.ts / (self.t1 - self.t0) * self.percent_gait - self.ts/(self.t1 - self.t0) * self.t0;
				# 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
				self.set_exo_current(max(NO_SLACK_CURRENT, A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau))))); #Commented out till output tested.
				#print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T1 Region : tau : ' + str(tau) )

			elif (self.percent_gait <= self.t2) : # the rising spline
				# 1 cout << "exoBoot :: runCollinsProfile : In t2 region" << endl;
				if (self.mode != CTRL_CURRENT) :
					self.set_controller (CTRL_CURRENT);

				tau = self.a1 * m.pow(self.percent_gait,3) + self.b1 * m.pow(self.percent_gait,2) + self.c1 * self.percent_gait + self.d1;
				# 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
				self.set_exo_current(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau)))); #Commented out till output tested.
				# print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T2 Region : tau : ' + str(tau) )

			elif (self.percent_gait <= self.t3) : # the falling spline
				# 1 cout << "exoBoot :: runCollinsProfile : In t3 region" << endl;
				if (self.mode != CTRL_CURRENT) :
					set_controller (CTRL_CURRENT);

				tau = self.a2 * m.pow(self.percent_gait,3) + self.b2 * m.pow(self.percent_gait,2) + self.c2 * self.percent_gait + self.d2;
				# 1 cout << "exoBoot :: runCollinsProfile : tau = " << tau << endl;
				self.set_exo_current(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau)))); #Commented out till output tested.
				#print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T3 Region : tau : ' + str(tau) )

			else : # go to the slack position if we aren't commanding a specific value
				tau = 0;
				# 1 cout << "exoBoot :: runCollinsProfile : Going Slack" << endl;
				# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				# !! if using go slack pick small proportional control !!
				# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				# self.go_slack();
				self.set_exo_current(NO_SLACK_CURRENT);  # just enough to keep a small tension in the cable
				#print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  T4 Region : tau : ' + str(tau) )
			#print ('exoBoot :: run_collins_profile : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  current_cmd (mA) : ' + str(A_TO_MA(self.ankle_torque_to_current(NM_TO_NMM(tau)))))

	def percent_gait_calc(self) :
		if (-1 != self.expected_duration)  : # if the expected duration is set calculate the percent gait
			self.percent_gait = 100 * (self.data_current[self.idx_time] - self.heelstrike_timestamp_current) / self.expected_duration;

		if (100 < self.percent_gait) : # if it has gone past 100 just hold 100
			self.percent_gait = 100;
		#print ('exoBoot :: percent_gait_calc : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  percent_gait : ' + str(self.percent_gait) )

	def update_expected_duration(self) :
		# TODO : In addition to checking that the step time is within a range, also check the the time it is armed is within the typical time.  Common errors occur from short spikes in acceleration that can have a close frequency.

		step_time = self.heelstrike_timestamp_current - self.heelstrike_timestamp_previous
		#armed_time = 0
		#if self.armed_timestamp != -1 :
			#armed_time = self.heelstrike_timestamp_current - self.armed_timestamp
		if (-1 == self.heelstrike_timestamp_previous) : # if it is the first time running just record the timestamp
			self.heelstrike_timestamp_previous = self.heelstrike_timestamp_current;
			return;
		if  (-1 in self.past_gait_times) : # if all the values haven't been replaced
			self.past_gait_times.insert(0, step_time);  # insert the new value at the beginning
			self.past_gait_times.pop(); # remove the last value
		elif ((step_time <= 1.75 * max(self.past_gait_times)) and (step_time >= 0.25 * min(self.past_gait_times))):# and (armed_time > ARMED_DURATION_PERCENT * self.expected_duration)): # a better check can be used.  If the person hasn't stopped or the step is good update the vector.
		# !!!THE ARMED TIME CHECK STILL NEEDS TO BE TESTED!!!
			self.past_gait_times.insert(0, step_time);  # insert the new value at the beginning
			self.past_gait_times.pop(); # remove the last value
			# TODO: Add rate limiter for change in expected duration so it can't make big jumps
			self.expected_duration = sum(self.past_gait_times)/len(self.past_gait_times);  # Average to the nearest ms

		#print ('exoBoot :: update_expected_duration : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ '  expected_duration : ' + str(self.expected_duration) )

	def clear_gait_estimate(self) :
		self.past_gait_times = [-1] * NUM_GAIT_TIMES_TO_AVERAGE	# store the most recent gait times
		self.expected_duration = -1	# current estimated gait duration


	def check_for_heelstrike(self) :
		# the trigger on the inversion of the leg is one method.
		# can also use spikes in acceleration (X seems to be best candidate but may not work well for slower gaits with smaller impacts)
		# Other candidates also possible.
		triggered = False
		armed_time = 0

		if self.armed_timestamp != -1 :
			armed_time = self.data_current[self.idx_time] - self.armed_timestamp

		if ((not self.heelstrike_armed) and (self.data_current[self.idx_gyro_z] >= self.segmentation_arm_threashold) and (self.data_current[self.idx_gyro_x] <= -50) ):
		#if ((not self.heelstrike_armed) and self.data_current[self.idx_gyro_z] >= self.segmentation_arm_threashold) :
			self.heelstrike_armed = True
			self.armed_timestamp = self.data_current[self.idx_time]
		if (self.heelstrike_armed and (self.data_current[self.idx_gyro_z] <= self.segmentation_trigger_threashold) and (self.data_current[self.idx_gyro_x] >= 0) ) :
		#if (self.heelstrike_armed and (self.data_current[self.idx_gyro_z] <= self.segmentation_trigger_threashold) ) :
			self.heelstrike_armed = False
			self.armed_timestamp = -1
			if  (armed_time > ARMED_DURATION_PERCENT/100 * self.expected_duration) :
				triggered = True

		self.segmentation_trigger = triggered
		#print ('exoBoot :: check_for_heelstrike : side : ' + ('LEFT' if self.side == LEFT else 'RIGHT')	+ ' : GyroZ : ' + str(self.data_current[self.idx_gyro_z] ) + ' : heelstrike_armed : ' + str(self.heelstrike_armed) + ' : triggered : ' + ('True' if triggered else 'False'))

	def __del__(self):
		# TODO: make where it can take single or multiple values
		#if self.should_log :
		if self.data_file != None :
			self.data_file.close()
# need to change api here
		fxStopStreaming(self.dev_id)
		closePort(self.idx)
