from .sp_utils import Avantes
import time 
import ctypes 
from tkinter import *
import numpy as np

class Spectrometer(): 
    def __init__(self, ws): 
        self.root = Tk()
        self.hWnd = self.root.winfo_id()
        self.w_active = True
        self.INTEGRATION_TIME = 0.1
        self.ws = ws
        

    def set_integration_time(self, it): 
        self.INTEGRATION_TIME = it

    def get_integration_time(self):
        return self.INTEGRATION_TIME

    def setup_equipment(self): 
        try:
            self.ava = Avantes(r"C:\Users\danie\Desktop\AvaspecSocket\backendWebsocket\spectrometer\DLL\avs.dll", ws = self.ws)
            result_init = self.ava.init()
            self.ava.activate()
            num_pix_result, num_pixels = self.ava.get_num_pixels()
            wl_result, self.wl = self.ava.get_lambda()
        except:
            print("An error occured while trying to communicate with the Spectrometer.")
            self.ws.close()
    
    def perform_measurement(self, num_measurements): 
        measurements = {}
        for i in range(num_measurements):
            it = int(self.INTEGRATION_TIME*1000)
            m_result = self.ava.measure(it=it,window_handle=self.hWnd)
            print("Result from AVS_Measure: {}".format(m_result))
            w_active = True
            while w_active:
                get_data_result, values = self.ava.get_data()
                if get_data_result == 0: 
                    w_active = False
                    measurements[i] ={
                        "wl": self.wl.tolist(), 
                        "spectra": values.tolist()
                    }
                time.sleep(self.INTEGRATION_TIME)
        self.root.quit()
        self.root.destroy()
        self.ava.disconnect()
        return measurements




# ava = Avantes(r".\DLL\avs.dll")
# print("Result from AVS_Init: {}".format(result_init))

# ava.activate()
# print("The device was successfully activated")

# config_result, params = ava.get_parameter()
# print("Result from AVS_GetParameter: {}".format(config_result))

# num_pix_result, num_pixels = ava.get_num_pixels()
# print("Result from AVS_GetNumPixels: {}".format(num_pix_result))

# wl_result, wl = ava.get_lambda()
# print("Result from AVS_GetLambda: {}".format(wl_result))

# (gain_result, gain) = ava.get_gain()
# print("Result from AVS_GetGain: {}".format(gain_result))

# (offset_result, offset) = ava.get_offset()
# print("Result from AVS_GetOffset: {}".format(offset_result))



# for i in range(num_measurements):
#     m_result = ava.measure(window_handle=hWnd)
#     print("Result from AVS_Measure: {}".format(m_result))
#     pixel_values = None
#     w_active = True
#     while w_active:
#         get_data_result, values = ava.get_data()
#         if get_data_result == 0: 
#             pixel_values =  values
#             w_active = False
#         time.sleep(INTEGRATION_TIME)
  

# root.quit()
# root.destroy()
# ava.disconnect()


# df = pd.DataFrame(data = list(zip(wl,pixel_values)), columns=['Wavelength', 'Counts'])
# df.to_excel("spectrum.xlsx")