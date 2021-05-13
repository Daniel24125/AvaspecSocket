import os
import sys
import time 
import ctypes 
import numpy as np
from .sp_utils import Avantes

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from errors import SpectrometerException
 

class Spectrometer(): 
    def __init__(self,path): 
        self.wl = None
        self.ava = Avantes(path)

    def set_integration_time(self, it): 
        self.INTEGRATION_TIME = it

    def get_integration_time(self):
        return self.INTEGRATION_TIME

    def get_wavelength(self): 
        return self.wl if self.wl else {"error": True}

    def init_spectrometer(self): 
        result = self.ava.init()

    def setup_equipment(self): 
        print("Starting equipment setup")
        result_init = self.ava.init()
        self.ava.activate()
        print("Equipment setup finished")
       
    def get_gain(self): 
        return self.ava.get_gain()
    
    def perform_measurement(self, num_measurements, it=100): 
        from tkinter import Tk
        root = Tk()
        hWnd = root.winfo_id()
        wl_result, self.wl = self.ava.get_lambda()
        # measurements = {}
        measurements = np.zeros(2048)
        for i in range(num_measurements):
            m_result = self.ava.measure(it=it,window_handle=hWnd)
            print(f"Result from AVS_Measure {i}: {m_result}")
            finished_measurement = False
            for _ in range(3):
                get_data_result, values = self.ava.get_data()
                if get_data_result == 0: 
                    finished_measurement = True
                    measurements = measurements + values
                time.sleep(0.3)
            if not finished_measurement:
                return (True, None, None)
        root.quit()
        root.destroy()
        measurements = measurements/num_measurements
        np.around(measurements, 2)
        return (False, np.around(self.wl,2).tolist()[:1454], measurements.tolist()[:1454])

    def disconnect(self): 
        self.ava.disconnect()


