import serial
import csv
import time
import numpy as np
import warnings
import serial
import serial.tools.list_ports
from datetime import datetime
# log serial port data from arduino to csv file
# This code can be used in later on real-time data processing


arduino_ports = [
    p.device
    for p in serial.tools.list_ports.comports()
    if 'Arduino' in p.description
]
if not arduino_ports:
    raise IOError("No Arduino found")
if len(arduino_ports) > 1:
    warnings.warn('Multiple Arduinos found - using the first')

Arduino = serial.Serial(arduino_ports[0], baudrate=57600)
# Arduino = serial.Serial(arduino_ports[0], baudrate=2000000)
Arduino.flush()
Arduino.reset_input_buffer()
# filename = datetime.now().strftime('%m_%d_%H_%M_ZJ_pel_IMU_Rizzoli_squat_afterwalk_01.csv')
filename = datetime.now().strftime('test1.csv')
with open(filename, 'w+', newline='') as outfile:
    outfileWrite = csv.writer(outfile)
    outfileWrite.writerow(['Sample Number', 'Pin 1', 'Pin 2', 'Pin 3', 'Pin 4'])
    print("start recording")
    time.sleep(1)
    while True:
        while (Arduino.inWaiting()==0):
            pass
        try:
            data = Arduino.readline()
            dataarray = data.decode().rstrip().split("\t")
            Arduino.reset_input_buffer()
            # print(dataarray)
            sample_num = float(dataarray[0])   # sample number
            rawdata1 = float(dataarray[1])   # raw value of the textile sensor 1
            rawdata2 = float(dataarray[2])   # raw value of the textile sensor 2
            rawdata3 = float(dataarray[3])   # raw value of the textile sensor 3
            rawdata4 = float(dataarray[4])   # raw value of the textile sensor 4
            #filterdata1 = float (dataarray[2])   # filtered value using an exponential filter
            #filterdata2 = float (dataarray[3])   # filtered value using an exponential filter
            # t = time.perf_counter()
            # print(rawdata4)
            #print ("time", time, "raw data", rawdata, "filterdata", filterdata)
            outfileWrite.writerow([sample_num, rawdata1, rawdata2, rawdata3, rawdata4])
            #outfileWrite.writerow([t, rawdata1,filterdata1, rawdata2, filterdata1,])
        except (KeyboardInterrupt, SystemExit,IndexError,ValueError):
            pass

outfile.close()
