#C:/Users/sgfki/AppData/Local/Programs/Python/Python37 python
import os, sys
thisdir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(thisdir)

from flexsea import flexsea as flex
from flexsea import fxUtils as fxu
from flexsea import fxEnums as fxe
from test_new_def_v2 import *
import time
# import pandas as p
fxs = flex.FlexSEA()

ZEROING_CURRENT = 400 # mA

def zero_boots (leftExo,rightExo) :

	# the gains need to be defined before this is called

	leftExo.set_controller(fxe.FX_CURRENT)
	rightExo.set_controller(fxe.FX_CURRENT)

	leftExo.set_exo_current(ZEROING_CURRENT)
	rightExo.set_exo_current(ZEROING_CURRENT)

	time.sleep(1) # wait a second

	"""
	# may not be necessary for new dephy api

	# leftExo.zero_encoders()
	# rightExo.zero_encoders()
	# print("zeroBoots : leftExo.ankle_ticks_offset = " + str(leftExo.ankle_ticks_offset) )
	# print("zeroBoots : rightExo.ankle_ticks_offset = " + str(rightExo.ankle_ticks_offset) )
	"""


	#leftExo.set_exo_current(0)
	#rightExo.set_exo_current(0)
	#leftExo.set_controller(fxe.FX_NONE)
	#rightExo.set_controller(fxe.FX_NONE)


def zero_boot (exo) :

	# the gains need to be defined before this is called

	exo.set_controller(fxe.FX_CURRENT)


	exo.set_exo_current(ZEROING_CURRENT)

	time.sleep(2) # wait a second

	"""
	# not sure if this will update the original encoder offsets or if this is a new instance of the boots.
	# may not be necessary for new dephy api
	# exo.zero_encoders()
	# print("zeroBoots : exo.ankle_ticks_offset = " + str(exo.ankle_ticks_offset) )
	"""

	# exo.set_exo_current(0)
	# exo.set_controller(fxe.FX_NONE)


def main():
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
	idx = 0 # keep idx = 0 for left boot
	print('creating left boot')
	left_boot =  ExoBoot(ports[0], int(baud_rate), idx, log_level = 4, shouldLog = True, frequency = 100) # recent change the first line is now the baud rate
	idx +=1 #keep idx = 1 for right boot
	right_boot = ExoBoot(ports[1], int(baud_rate), idx, log_level = 4, shouldLog = True, frequency = 100)

	if False : # ( (not left_boot.id) or (not right_boot.id) ):
		print("At least one boot is missing")
	else :
		# TODO: Add in gains for current and position
		left_boot.define_current_gains(40,400,128)
		right_boot.define_current_gains(40,400,128)

		zero_boots(left_boot, right_boot)
		#zero_boot(left_boot);
		#print("main : left_boot.ankle_ticks_offset = " + str(left_boot.ankle_ticks_offset ) )

		print("OPTIMIZED CONTROL LAW ----------")
		user_mass = 75
		rspg = 0
		opg = 23
		ppg = 38
		spg = 48
		ot = 0.056 # set onset current to be 200mA
		npt = 0.025/3 # set peak current to be optimized one

		# print("OPTIMIZED CONTROL LAW ----------")
		# user_mass = data.at[0,'gen3']
		# rspg = data.at[1,'gen3']
		# opg = data.at[4,'gen3']
		# ppg = data.at[5,'gen3']
		# spg = data.at[6,'gen3']
		# ot = data.at[2,'gen3'] # set onset current to be 400mA
		# npt = data.at[3,'gen3']

		print ('user mass: '+ str(user_mass))
		print ('rspg: '+ str(rspg))
		print ('opg: '+ str(opg))
		print ('ppg: '+ str(ppg))
		print ('spg: '+ str(spg))
		print ('ot: '+ str(ot))
		print ('npt: '+ str(npt))

		left_boot.init_collins_profile(mass = user_mass, ramp_start_percent_gait = rspg, onset_percent_gait = opg, peak_percent_gait = ppg, stop_percent_gait = spg, onset_torque = ot, normalized_peak_torque = npt)	# initialize the Zhang/Collins profile
		right_boot.init_collins_profile(mass = user_mass, ramp_start_percent_gait = rspg, onset_percent_gait = opg, peak_percent_gait = ppg, stop_percent_gait = spg, onset_torque = ot, normalized_peak_torque = npt)	# initialize the Zhang/Collins profile

		#left_boot.set_controller(CTRL_CURRENT)
		#right_boot.set_controller(CTRL_CURRENT)

		start_time = 0

		print('starting loop')
		try:
			while True:
				# put main code loop here it will stop with a CTRL+c
				# while ((time.monotonic()-start_time) <= (1/left_boot.frequency)) : # if you run boot interface loop faster than the dephy frequency the cue will fill up and it will take a long time shut down and to have the data come in.  This just waits till the boot period has passed then runs.
					# pass
				# start_time = time.monotonic()

				left_boot.read_data()
				right_boot.read_data()

				left_boot.run_collins_profile(external_read = True)	# apply the torque appropriate to the Zhang/Collins profile
				right_boot.run_collins_profile(external_read = True)	# apply the torque appropriate to the Zhang/Collins profile

				time.sleep(1/left_boot.frequency)  # Lazy alternative to the waiting while loop but will run a bit slower.

		except KeyboardInterrupt:
			print("KeyboardInterrupt has been caught.")


	left_boot.set_exo_current(0)
	right_boot.set_exo_current(0)

	time.sleep(.3)

	left_boot.set_controller(fxe.FX_NONE)
	right_boot.set_controller(fxe.FX_NONE)

	time.sleep(.3)
	del left_boot
	time.sleep(.3)
	del right_boot
	# cleanupPlanStack()



main()
