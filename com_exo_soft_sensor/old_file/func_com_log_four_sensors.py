import serial
import time
import numpy as np
import warnings
import serial
import serial.tools.list_ports
from datetime import datetime
# log serial port data from arduino to csv file
# This code can be used in later on real-time data processing


def obtain_softsensor_data():
    while (Arduino.inWaiting() == 0):
        pass
    try:
        data = Arduino.readline()
        dataarray = data.decode().rstrip().split("\t")
        Arduino.reset_input_buffer()
        # print(dataarray)
        rawdata1 = float(dataarray[0])  # raw value of the textile sensor 1
        rawdata2 = float(dataarray[1])  # raw value of the textile sensor 2
        rawdata3 = float(dataarray[2])  # raw value of the textile sensor 3
        rawdata4 = float(dataarray[3])  # raw value of the textile sensor 4
        return rawdata1, rawdata2, rawdata3, rawdata4
    except (KeyboardInterrupt, SystemExit,IndexError,ValueError):
        pass


