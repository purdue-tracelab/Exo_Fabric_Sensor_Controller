import numpy as np
import warnings
import serial
import serial.tools.list_ports

class Arduino:

    def __init__(self):
        print('connecting to arduino')
        self.arduino = self._connectToArduino()  # connect to the boot
        self.obtain_softsensor_data()  # read in the current data from the system

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
            rawdata1 = float(dataarray[0])  # raw value of the textile sensor 1
            rawdata2 = float(dataarray[1])  # raw value of the textile sensor 2
            rawdata3 = float(dataarray[2])  # raw value of the textile sensor 3
            rawdata4 = float(dataarray[3])  # raw value of the textile sensor 4
            return rawdata1, rawdata2, rawdata3, rawdata4
        except (KeyboardInterrupt, SystemExit, IndexError, ValueError):
            pass