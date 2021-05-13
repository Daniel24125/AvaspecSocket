"""
Wrapper around the ``avaspec.dll`` SDK from Avantes.

The wrapper was written using v9.7.0.0 of the SDK.
"""
from ctypes import *
from enum import IntEnum
import numpy as np
import sys
import time
import json
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from errors import SpectrometerException


class Avantes():
    def __init__(self, path):
        self._handle = None
        self.sdk = WinDLL(path)
        self.MeasConfigType = MeasConfigType  #: :class:`MeasConfigType`
        self.OemDataType = OemDataType  #: :class:`OemDataType`
        self.DeviceConfigType = DeviceConfigType  #: :class:`DeviceConfigType`
        self.path = path

        functions = {
            'AVS_Init': (c_int32, self._err_check,
                [('a_Port', c_int16)]),
            'AVS_Done': (c_int32, self._err_check,
                []),
            'AVS_GetList': (c_int32, self._err_check,
                   [('a_ListSize', c_uint32),
                    ('a_pRequiredSize', POINTER(c_uint32)),
                    ('a_pList', POINTER(AvsIdentityType))]),
            'AVS_Activate': (c_int32, self._err_check,
                [('a_pDeviceId', POINTER(AvsIdentityType))]),        
            'AVS_Deactivate': (c_bool, self._check_bool,
                [('a_hDevice', c_int32)]),
            'AVS_Register': (c_bool, self._check_bool,
                [('a_Hwnd', c_void_p)]),
            'AVS_Measure': (c_int32, self._err_check,
                [
                    ('a_IntegrationTime', c_uint),
                    ('a_Scans', c_uint),
                    ('a_hWnd', c_void_p)
                ]),
            'AVS_GetGain': (c_int32, self._err_check,
                [
                    ('a_hDevice', c_int32),
                    ('a_pGain', POINTER(c_float))
                ]),
            'AVS_GetOffset': (c_int32, self._err_check,
                [
                    ('a_hDevice', c_int32),
                    ('a_pOffset', POINTER(c_float))
                ]),
       
            'AVS_GetSaturatedPixels': (c_int32, self._err_check,
                [('a_hDevice', c_int32),
                    ('a_pSaturated', POINTER(c_uint))]),
            'AVS_GetLambda': (c_int32,self. _err_check,
                     [('a_hDevice', c_int32),
                    ('a_pWaveLength', POINTER(c_double))]),
            'AVS_GetNumPixels': (c_int32, self._err_check,
                [('a_pNumPixels', POINTER(c_uint16))]),
            'AVS_GetParameter': (c_int32, self._err_check,
                [('a_pDeviceParm', POINTER(DeviceConfigType))]),
            'AVS_SetParameter': (c_int32, self._err_check,
                [('a_hDevice', c_int32),
                    ('a_pDeviceParm', POINTER(DeviceConfigType))]),
            'AVS_GetDLLVersion': (c_int32, self._err_check,
                [('a_pVersionString', c_char_p)]),
        
            'AVS_GetDigIn': (c_int32, self._err_check,
                [('a_hDevice', c_int32),
                    ('a_DigInId', c_ubyte),
                    ('a_pDigIn', POINTER(c_ubyte))]),
        }
        
        for key, value in functions.items():
            attr = getattr(self.sdk, key)
            attr.restype, attr.errcheck = value[:2]
            attr.argtypes = [typ for _, typ in value[2]]


    def _get_dll_library(self): 
        return cdll

    def raise_exception(self, msg, error_name=None, code=None): 
        raise SpectrometerException(error_code=code, error_msg=msg, error_name=error_name)
        

    def _err_check(self, result, func, arguments):
        if result < 0:
            error_name, msg = ERROR_CODES.get(result, ('UNKNOWN_ERROR', 'Unknown error'))
            self.raise_exception(msg=msg, error_name=error_name, code=result )
        return result

    def _check_bool(self, result, func, arguments):
        if not result:
            self.raise_exception(msg='The {} function returned False'.format(func))
         
    def activate(self):
        """Activates the spectrometer for communication.
            Name         : AVS_Activate(
                AvsIdentityType*    a_pDeviceId
            )

            Description  : Activates selected spectrometer for communication

            Parameters   : a_pDeviceId   : AvsIdentity of desired spectrometer

            Returns      : AvsHandle     : handle to be used in subsequent calls
                            INVALID_HANDLE_VALUE  : in case of error
        """
        out = self.get_list()
        if len(out) < 1:
            self.raise_exception('Cannot activate. No devices found.')
        for item in out:
            self._handle = self.sdk.AVS_Activate(item)
            return 

    def deactivate(self):
        """Closes communication with the spectrometer."""
        if self._handle:
            self.sdk.AVS_Deactivate(self._handle)
            self._handle = None

    def get_dll_version(self):
        """Get the DLL version number.

        Returns
        -------
        :class:`str`
            The DLL version number
        """
        version = create_string_buffer(255)
        self.sdk.AVS_GetDLLVersion(version)
        return version.value.decode()

    def get_digital_in(self, digital_id):
        """Get the status of the specified digital input.

        Parameters
        ----------
        digital_id : :class:`int`
            The identifier of the digital input to get.

            * AS5216:

                * 0 = DI1 = Pin 24 at 26-pins connector
                * 1 = DI2 = Pin 7 at 26-pins connector
                * 2 = DI3 = Pin 16 at 26-pins connector

            * Mini:

                * 0 = DI1 = Pin 7 on Micro HDMI = Pin 5 on HDMI terminal
                * 1 = DI2 = Pin 5 on Micro HDMI = Pin 3 on HDMI Terminal
                * 2 = DI3 = Pin 3 on Micro HDMI = Pin 1 on HDMI Terminal
                * 3 = DI4 = Pin 1 on Micro HDMI = Pin 19 on HDMI Terminal
                * 4 = DI5 = Pin 4 on Micro HDMI = Pin 2 on HDMI Terminal
                * 5 = DI6 = Pin 2 on Micro HDMI = Pin 14 on HDMI Terminal

            * AS7010:

                * 0 = DI1 = Pin 24 at 26-pins connector
                * 1 = DI2 = Pin 7 at 26-pins connector
                * 2 = DI3 = Pin 16 at 26-pins

        Returns
        -------
        :class:`int`
            The digital input value.

        Raises
        ------
        ~msl.equipment.exceptions.AvantesError
            If there was an error.
        """
        din = c_ubyte()
        self.sdk.AVS_GetDigIn(self._handle, digital_id, din)
        return din.value

    def get_handle_from_serial(self, serial=None):
        """Get the handle ID for the specified serial number.

        Parameters
        ----------
        serial : :class:`str`
            The serial number. Default is to get the status for this object.

        Returns
        -------
        :class:`int`
            The handle.

        Raises
        ------
        ~msl.equipment.exceptions.AvantesError
            If there was an error.
        """
        if serial is None:
            serial = self.equipment_record.serial.encode()
        elif not isinstance(serial, bytes):
            serial = serial.encode()
        return self.sdk.AVS_GetHandleFromSerial(serial)

    def get_list(self, nmax=1):
        """Returns device information for each spectrometer that is connected.
             
            Name         : AVS_GetList(
                unsigned int        a_ListSize,
                unsigned int*       a_pRequiredSize,
                AvsIdentityType*    a_pList
            )

            Description : Returns device information for each spectrometer connected
                            to the ports indicated at AVS_Init
            
            Parameters  : a_ListSize   : number of bytes allocated by the caller to
                                            store the a_pList data
                        a_pRequiredSize : Number of bytes needed to store information
                        a_pList         : pointer to allocated buffer to store information
            
            Returns     : int           : number of devices in list (if 0 and a_pRequiredSize > a_ListSize, then allocate larger buffer)
        """
        size = nmax * sizeof(AvsIdentityType)
        required_size = c_uint32()
        self.types = (AvsIdentityType * nmax)()
        self.sdk.AVS_GetList.argtypes = [c_uint32, POINTER(c_uint32), POINTER(AvsIdentityType)]
        n = self.sdk.AVS_GetList(size, required_size, self.types)
        if n == 0: 
            self.raise_exception(msg='No device found.')
        if n < 0:
            error_name, msg = ERROR_CODES.get(n, ('UNKNOWN_ERROR', 'Unknown error'))
            self.raise_exception(msg=msg, error_name=error_name, code = n)

        return [self.types[i] for i in range(n)]

    def done(self):
        """Closes communication and releases internal storage."""
        self.sdk.AVS_Done()

    def disconnect(self):
        """Closes communication with the spectrometer."""
        self.deactivate()
        result_done = self.done()
        print(f"Result done: {result_done}")

    def get_lambda(self, channel=0):
        """Returns the wavelength values corresponding to the pixels if available.

            Name: AVS_GetLambda(
                unsigned char   a_Channel,
                double*         a_pWaveLength
            )
            
            Description  : Returns the wavelength values corresponding to the pixels
            
            Parameters   : a_Channel : number of channel
            
            Returns      : integer       : 0, successfully started
                                            error code on error
                            a_pWaveLength : pointer to array of doubles,
                                            array size equal to number of pixels
            
            Remark(s)    : array size not checked
        """
        values = np.zeros(MAX_NR_PIXELS, dtype=np.double)
        result = self.sdk.AVS_GetLambda(channel, values.ctypes.data_as(POINTER(c_double)))
        return (result, values[values > 0])

    def get_parameter(self):
        """Returns the device information of the spectrometer.
            Name         : AVS_GetParameter
            
            Description  : Returns the device parameter structure
            
            Parameters   : DeviceParmType* a_pDeviceParm
            
            Returns      : integer         : 0, info available
                                              error code on error
                            a_pDeviceParm   : pointer to structure for device information
        """
        dct = DeviceConfigType()
        result = self.sdk.AVS_GetParameter(dct)
        return (result, dct)

    def get_saturated_pixels(self):
        """Returns, for each pixel, if a pixel was saturated (1) or not (0).

        Name         : AVS_GetSaturatedPixels

        Description  : Returns for each pixel the number of scans (out of a total
                        of NrOfAverage scans) for which the pixel was saturated

        Parameters   : a_Channel : number of channel

        Returns      : integer       : 0, successfully started
                                        error code on error
                        a_pSaturated  : pointer to array of integers containing
                                        number of scans saturated,
                                        array size equal to number of pixels

        Remark(s)    : array size not checked
        """
        values = c_uint()
        result = self.sdk.AVS_GetSaturatedPixels(self._handle, values)
        return (result, values)

    def get_version_info(self):
        """Returns software version information.

        Returns
        -------
        :class:`str`
            FPGA software version.
        :class:`str`
            Firmware version.
        :class:`str`
            DLL version.

        Raises
        ------
        ~msl.equipment.exceptions.AvantesError
            If there was an error.
        """
        fpga = (c_ubyte * 16)()
        fm = (c_ubyte * 16)()
        dll = (c_ubyte * 16)()
        self.sdk.AVS_GetVersionInfo(self._handle, fpga, fm, dll)
        return [string_at(addressof(obj)).decode() for obj in [fpga, fm, dll]]

    def get_data(self, channel=0):
        """Returns the pixel values of the last performed measurement.

        Name         : AVS_GetScopeData
        (
            unsigned char   a_Channel,
            double*         a_pSpectrum
        )

        Description  : Returns the values for each pixel

        Parameters   : a_Channel : number of channel

        Returns      : integer       : 0, successfully started
                                        error code on error
                        a_pSpectrum   : pointer to array of doubles containing dark
                                        values, array size equal to number of pixels

        Remark(s)    : array size not checked
        """
        values = np.zeros(MAX_NR_PIXELS, dtype=np.double)
        result = self.sdk.AVS_GetScopeData(channel,values.ctypes.data_as(POINTER(c_double)))
        return (result, values)

    def get_num_channels(self): 
        """ Returns the number of channels available
            Name         : AVS_GetNumChannels

            Description  : Returns the number of channels

            Parameters   : -

            Returns      : integer          : 0, number of channels available
                                              error code on error
                            a_pNrOfChannels : number of channels
        """
        num_channels = c_ubyte()
        self.sdk.AVS_GetNumChannels.argtypes = [POINTER(c_ubyte)]
        self.sdk.AVS_GetNumChannels(num_channels)
        return num_channels

    def get_num_pixels(self):
        """Returns the number of pixels of a spectrometer.
            Name         : AVS_GetNumPixels(a_pNrOfPixels)
            
            Description  : Returns the number of pixels
            
            Parameters   : unsigned int*    a_pNrOfPixels
            
            Returns      : integer       : 0, number of pixels available
                                           error code on error
                           a_pNrOfPixels : number of pixels
                    """
        n = c_uint16()
        self.sdk.AVS_GetNumPixels.argtypes = [ POINTER(c_uint16)]
        result = self.sdk.AVS_GetNumPixels(n)
        return (result, n.value)

    def init(self, port_id=0):
        """Initializes the communication interface with the spectrometers and the internal data structures.

            For Ethernet devices this function will create a list of available Ethernet spectrometers
            within all the network interfaces of the host.

            Description  : Tries to open com-port and ask spectrometer configuration

            Parameters   : a_COMPort   : -1, search for port to be used
                                        0, use USB port
                                        1-4, use COM port

            Returns      : integer     :  0..4, port successfully opened
                                        256..260 port opened, but device unconfigured
                                        -1, error occured

            Remark(s)    : Blocks application
        """
        ret = self.sdk.AVS_Init(port_id)
        if ret < 0:
            self.raise_exception(msg='No Avantes devices were found')
        return ret

    def measure(self, it=100, num_measurements=1, window_handle=None):
        """Starts measurement on the spectrometer.
            Name         : AVS_Measure(
                unsigned int    a_IntegrationTime,
                unsigned int    a_Scans,
                HWND            a_hWnd
            )

            Description  : Start measurement

            Parameters   : a_IntegrationTime : integration time in ms
                            a_Scans           : number of scans
                            a_hWnd            : handle of window to which ready message
                                                should be sent

            Returns      : integer : 0, successfully started
                                      error code on error
       

        """
        return self.sdk.AVS_Measure(it, num_measurements, window_handle)

    def register(self, handle):
        """Installs an application windows handle to which device
        attachment/removal messages have to be sent.

        Parameters
        ----------
        handle : :class:`ctypes.c_void_p`
            Application window handle.

        Raises
        ------
        ~msl.equipment.exceptions.AvantesError
            If there was an error.
        """
        self.sdk.AVS_Register(handle)

    def set_digital_out(self, port_id, value):
        """Sets the digital output value for the specified digital identifier.

        Parameters
        ----------
        port_id : :class:`int`
            Identifier for one of the 10 output signals:

            * AS5216:

                * 0 = DO1 = pin 11 at 26-pins connector
                * 1 = DO2 = pin 2 at 26-pins connector
                * 2 = DO3 = pin 20 at 26-pins connector
                * 3 = DO4 = pin 12 at 26-pins connector
                * 4 = DO5 = pin 3 at 26-pins connector
                * 5 = DO6 = pin 21 at 26-pins connector
                * 6 = DO7 = pin 13 at 26-pins connector
                * 7 = DO8 = pin 4 at 26-pins connector
                * 8 = DO9 = pin 22 at 26-pins connector
                * 9 = DO10 = pin 25 at 26-pins connector

            * Mini:

                * 0 = DO1 = Pin 7 on Micro HDMI = Pin 5 on HDMI terminal
                * 1 = DO2 = Pin 5 on Micro HDMI = Pin 3 on HDMI Terminal
                * 2 = DO3 = Pin 3 on Micro HDMI = Pin 1 on HDMI Terminal
                * 3 = DO4 = Pin 1 on Micro HDMI = Pin 19 on HDMI Terminal
                * 4 = DO5 = Pin 4 on Micro HDMI = Pin 2 on HDMI Terminal
                * 5 = DO6 = Pin 2 on Micro HDMI = Pin 14 on HDMI Terminal
                * 6 = Not used
                * 7 = Not used
                * 8 = Not used
                * 9 = Not used

            * AS7010:

                * 0 = DO1 =pin 11 at 26-pins connector
                * 1 = DO2 = pin 2 at 26-pins connector
                * 2 = DO3 = pin 20 at 26-pins connector
                * 3 = DO4 = pin 12 at 26-pins connector
                * 4 = DO5 = pin 3 at 26-pins connector
                * 5 = DO6 = pin 21 at 26-pins connector
                * 6 = DO7 = pin 13 at 26-pins connector
                * 7 = DO8 = pin 4 at 26-pins connector
                * 8 = DO9 = pin 22 at 26-pins connector
                * 9 = DO10 = pin 25 at 26-pins connector

        value : :class:`int`
            The value to be set (0 or 1).

        Raises
        ------
        ~msl.equipment.exceptions.AvantesError
            If there was an error.
        """
        self.sdk.AVS_SetDigOut(self._handle, port_id, value)

    def set_parameter(self, parameter):
        """Overwrites the device configuration.

        Please note that :class:`OemDataType` is part of the DeviceConfigType in EEPROM (see
        section 3.5 of DLL manual). Precautions must be taken to prevent OEM data overwrites
        when using :meth:`.set_parameter` method together with :meth:`.set_oem_parameter`.

        Parameters
        ----------
        parameter : :class:`.DeviceConfigType`
            The device parameters.

        Raises
        ------
        ~msl.equipment.exceptions.AvantesError
            If there was an error.
        """
        if not isinstance(parameter, DeviceConfigType):
            self.raise_exception(msg='Must pass in a DeviceConfigType object')
        self.sdk.AVS_SetParameter(self._handle, parameter)

    def get_gain(self): 
        gain = c_float()
        result = self.sdk.AVS_GetGain(self._handle, gain)
        return (result, gain.value)

    def get_offset(self): 
        offset = c_float()
        result = self.sdk.AVS_GetOffset(self._handle, offset)
        return (result, offset.value)

    # def get_saturated_pixels(self): 
    #     sat_pixels = c_uint()
    #     result = self.sdk.AVS_GetSaturatedPixels(self._handle, sat_pixels)
    #     return (result, sat_pixels)

WM_MEAS_READY = 0x8001
SETTINGS_RESERVED_LEN = 9720
INVALID_AVS_HANDLE_VALUE = 1000
USER_ID_LEN = 64
AVS_SERIAL_LEN = 10
MAX_TEMP_SENSORS = 3
ROOT_NAME_LEN = 6
VERSION_LEN = 16
AVASPEC_ERROR_MSG_LEN = 8
AVASPEC_MIN_MSG_LEN = 6
OEM_DATA_LEN = 2048
NR_WAVELEN_POL_COEF = 5
NR_NONLIN_POL_COEF = 8
MAX_VIDEO_CHANNELS = 2
NR_DEFECTIVE_PIXELS = 30
MAX_NR_PIXELS = 2048
NR_TEMP_POL_COEF = 5
NR_DAC_POL_COEF = 2
SAT_PEAK_INVERSION = 2
SW_TRIGGER_MODE = 0
HW_TRIGGER_MODE = 1
SS_TRIGGER_MODE = 2
EXTERNAL_TRIGGER = 0
SYNC_TRIGGER = 1
EDGE_TRIGGER_SOURCE = 0
LEVEL_TRIGGER_SOURCE = 1
ILX_FIRST_USED_DARK_PIXEL = 2
ILX_USED_DARK_PIXELS = 14
ILX_TOTAL_DARK_PIXELS = 18
TCD_FIRST_USED_DARK_PIXEL = 0
TCD_USED_DARK_PIXELS = 12
TCD_TOTAL_DARK_PIXELS = 13
HAMS9840_FIRST_USED_DARK_PIXEL = 0
HAMS9840_USED_DARK_PIXELS = 8
HAMS9840_TOTAL_DARK_PIXELS = 8
HAMS10420_FIRST_USED_DARK_PIXEL = 0
HAMS10420_USED_DARK_PIXELS = 4
HAMS10420_TOTAL_DARK_PIXELS = 4
HAMS11071_FIRST_USED_DARK_PIXEL = 0
HAMS11071_USED_DARK_PIXELS = 4
HAMS11071_TOTAL_DARK_PIXELS = 4
HAMS7031_FIRST_USED_DARK_PIXEL = 0
HAMS7031_USED_DARK_PIXELS = 4
HAMS7031_TOTAL_DARK_PIXELS = 4
HAMS11155_TOTAL_DARK_PIXELS = 20
MIN_ILX_INTTIME = 1.1
MILLI_TO_MICRO = 1000
NR_DIGITAL_OUTPUTS = 13
NR_DIGITAL_INPUTS = 13
NTC1_ID = 0
NTC2_ID = 1
TEC_ID = 2
NR_ANALOG_OUTPUTS = 2
ETH_CONN_STATUS_CONNECTING = 0
ETH_CONN_STATUS_CONNECTED = 1
ETH_CONN_STATUS_CONNECTED_NOMON = 2
ETH_CONN_STATUS_NOCONNECTION = 3

ERROR_CODES = {
    -1: ('ERR_INVALID_PARAMETER',
         'Function called with invalid parameter value.'),
    -2: ('ERR_OPERATION_NOT_SUPPORTED',
         'Function not supported (e.g. use 16bit ADC mode with 14bit ADC hardware)'),
    -3: ('ERR_DEVICE_NOT_FOUND',
         'Opening communication failed or time-out during communication occurred.'),
    -4: ('ERR_INVALID_DEVICE_ID',
         'AvsHandle is unknown in the DLL'),
    -5: ('ERR_OPERATION_PENDING',
         'Function is called while result of previous call to AVS_Measure() is not received yet'),
    -6: ('ERR_TIMEOUT',
         'No answer received from device'),
    -7: ('Reserved',
         ''),
    -8: ('ERR_INVALID_MEAS_DATA',
         'No measurement data is received at the point AVS_GetScopeData() is called'),
    -9: ('ERR_INVALID_SIZE',
         'Allocated buffer size too small'),
    -10: ('ERR_INVALID_PIXEL_RANGE',
          'Measurement preparation failed because pixel range is invalid'),
    -11: ('ERR_INVALID_INT_TIME',
          'Measurement preparation failed because integration time is invalid (for selected sensor)'),
    -12: ('ERR_INVALID_COMBINATION',
          'Measurement preparation failed because of an invalid combination of parameters, e.g. integration time of 600000 and Navg > 5000'),
    -13: ('Reserved',
          ''),
    -14: ('ERR_NO_MEAS_BUFFER_AVAIL',
          'Measurement preparation failed because no measurement buffers available'),
    -15: ('ERR_UNKNOWN',
          'Unknown error reason received from spectrometer'),
    -16: ('ERR_COMMUNICATION',
          'Error in communication or Ethernet connection failure'),
    -17: ('ERR_NO_SPECTRA_IN_RAM',
          'No more spectra available in RAM, all read or measurement not started yet'),
    -18: ('ERR_INVALID_DLL_VERSION',
          'DLL version information could not be retrieved'),
    -19: ('ERR_NO_MEMORY',
          'Memory allocation error in the DLL'),
    -20: ('ERR_DLL_INITIALISATION',
          'Function called before AVS_Init() is called'),
    -21: ('ERR_INVALID_STATE',
          'Function failed because AvaSpec is in wrong state, e.g. AVS_Measure() without calling AVS_PrepareMeasurement() first'),
    -22: ('ERR_INVALID_REPLY',
          'Reply is not a recognized protocol message'),
    -23: ('Reserved',
          ''),
    -24: ('ERR_ACCESS',
          'Error occurred while opening a bus device on the host, e.g. USB device access denied due to user rights'),
    -100: ('ERR_INVALID_PARAMETER_NR_PIXEL',
           'NrOfPixel in Device data incorrect'),
    -101: ('ERR_INVALID_PARAMETER_ADC_GAIN',
           'Gain Setting Out of Range'),
    -102: ('ERR_INVALID_PARAMETER_ADC_OFFSET',
           'OffSet Setting Out of Range'),
    -110: ('ERR_INVALID_MEASPARAM_AVG_SAT2',
           'Use of Saturation Detection Level 2 is not compatible with the Averaging function'),
    -111: ('ERR_INVALID_MEASPARAM_AVG_RAM',
           'Use of Averaging is not compatible with the StoreToRam function'),
    -112: ('ERR_INVALID_MEASPARAM_SYNC_RAM',
           'Use of the Synchronize setting is not compatible with the StoreToRam function'),
    -113: ('ERR_INVALID_MEASPARAM_LEVEL_RAM',
           'Use of Level Triggering is not compatible with the StoreToRam function'),
    -114: ('ERR_INVALID_MEASPARAM_SAT2_RAM',
           'Use of Saturation Detection Level 2 Parameter is not compatible with the StoreToRam function'),
    -115: ('ERR_INVALID_MEASPARAM_FWVER_RAM',
           'The StoreToRam function is only supported with firmware version 0.20.0.0 or later.'),
    -116: ('ERR_INVALID_MEASPARAM_DYNDARK',
           'Dynamic Dark Correction not supported'),
    -120: ('ERR_NOT_SUPPORTED_BY_SENSOR_TYPE',
           'Use of AVS_SetSensitivityMode() not supported by detector type'),
    -121: ('ERR_NOT_SUPPORTED_BY_FW_VER',
           'Use of AVS_SetSensitivityMode() not supported by firmware version'),
    -122: ('ERR_NOT_SUPPORTED_BY_FPGA_VER',
           'Use of AVS_SetSensitivityMode() not supported by FPGA version'),
    -140: ('ERR_SL_CALIBRATION_NOT_AVAILABLE',
           'Spectrometer was not calibrated for stray light correction'),
    -141: ('ERR_SL_STARTPIXEL_NOT_IN_RANGE',
           'Incorrect start pixel found in EEPROM'),
    -142: ('ERR_SL_ENDPIXEL_NOT_IN_RANGE',
           'Incorrect end pixel found in EEPROM'),
    -143: ('ERR_SL_STARTPIX_GT_ENDPIX',
           'Incorrect start or end pixel found in EEPROM'),
    -144: ('ERR_SL_MFACTOR_OUT_OF_RANGE',
           'Factor should be in range 0.0 - 4.0'),
    -1000: ('DEVICE_NOT_FOUND',
           'Opening communication failed or time-out during communication occurred. Please make sure your device is connected and try again'),
    -1001: ('MEAS_NOT_ALLOWED',
           'Measurement not allowed'),
    -1011: ('INVALID_MEAS_DATA',
           'No measurement data is received at the point AVS_GetScopeData() is called'),
    -1012: ('MEAS_INTERVAL_TOO_SHORT',
           'Measurement interval is too short'),
    -1013: ('INVALID_SMOOTHING_PARAM',
           'Invalid smooth parameter'),
    -1100: ('INVALID_CHANNEL_ID',
           'Invalid channel ID'),
    -1101: ('DEVICE_RET_UNKNOWN_ERROR',
           'Device unknown error'),
    -1102: ('INVALID_PASSWORD',
           'Wrong password'),
    -1103: ('ERROR_IN_COMMUNICATION',
           'There was an error in the communication'),
    -1104: ('INVALID_SPECTRA_NUMBER',
           'Invalid spectra number'),
    -1105: ('EXT_TRIGGER_ENABLED',
           'External trigger is enabled'),
    -1106: ('INVALID_DLL_VERSION',
           'You are using an invalid DLL version'),

}


class DeviceStatus(IntEnum):
    """DeviceStatus enum."""
    UNKNOWN = 0
    USB_AVAILABLE = 1
    USB_IN_USE_BY_APPLICATION = 2
    USB_IN_USE_BY_OTHER = 3
    ETH_AVAILABLE = 4
    ETH_IN_USE_BY_APPLICATION = 5
    ETH_IN_USE_BY_OTHER = 6
    ETH_ALREADY_IN_USE_USB = 7



class InterfaceType(IntEnum):
    """InterfaceType enum."""
    RS232 = 0
    USB5216 = 1
    USBMINI = 2
    USB7010 = 3
    ETH7010 = 4



class SensType(IntEnum):
    """SensType enum."""
    SENS_HAMS8378_256 = 1
    SENS_HAMS8378_1024 = 2
    SENS_ILX554 = 3
    SENS_HAMS9201 = 4
    SENS_TCD1304 = 5
    SENS_TSL1301 = 6
    SENS_TSL1401 = 7
    SENS_HAMS8378_512 = 8
    SENS_HAMS9840 = 9
    SENS_ILX511 = 10
    SENS_HAMS10420_2048X64 = 11
    SENS_HAMS11071_2048X64 = 12
    SENS_HAMS7031_1024X122 = 13
    SENS_HAMS7031_1024X58 = 14
    SENS_HAMS11071_2048X16 = 15
    SENS_HAMS11155_2048 = 16
    SENS_SU256LSB = 17
    SENS_SU512LDB = 18
    SENS_HAMS11638 = 21
    SENS_HAMS11639 = 22
    SENS_HAMS12443 = 23
    SENS_HAMG9208_512 = 24
    SENS_HAMG13913 = 25
    SENS_HAMS13496 = 26



class AvsIdentityType(Structure):
    """IdentityType Structure."""
    _pack_ = 1
    _fields_ = [
        ('SerialNumber', c_char * AVS_SERIAL_LEN),
        # ('UserFriendlyName', c_char * USER_ID_LEN),
        ('Status', c_ubyte)
    ]



class BroadcastAnswerType(Structure):
    """BroadcastAnswerType Structure."""
    _pack_ = 1
    _fields_ = [
        ('InterfaceType', c_ubyte),
        ('serial', c_ubyte * AVS_SERIAL_LEN),
        ('port', c_uint16),
        ('status', c_ubyte),
        ('RemoteHostIp', c_uint32),
        ('LocalIp', c_uint32),
        ('reserved', c_ubyte * 4)
    ]



class ControlSettingsType(Structure):
    """ControlSettingsType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_StrobeControl', c_uint16),
        ('m_LaserDelay', c_uint32),
        ('m_LaserWidth', c_uint32),
        ('m_LaserWaveLength', c_float),
        ('m_StoreToRam', c_uint16),
    ]



class DarkCorrectionType(Structure):
    """DarkCorrectionType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_Enable', c_ubyte),
        ('m_ForgetPercentage', c_ubyte),
    ]



class DetectorType(Structure):
    """DetectorType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_SensorType', c_ubyte),
        ('m_NrPixels', c_uint16),
        ('m_aFit', c_float * NR_WAVELEN_POL_COEF),
        ('m_NLEnable', c_bool),
        ('m_aNLCorrect', c_double * NR_NONLIN_POL_COEF),
        ('m_aLowNLCounts', c_double),
        ('m_aHighNLCounts', c_double),
        ('m_Gain', c_float * MAX_VIDEO_CHANNELS),
        ('m_Reserved', c_float),
        ('m_Offset', c_float * MAX_VIDEO_CHANNELS),
        ('m_ExtOffset', c_float),
        ('m_DefectivePixels', c_uint16 * NR_DEFECTIVE_PIXELS)
    ]



class SmoothingType(Structure):
    """SmoothingType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_SmoothPix', c_uint16),
        ('m_SmoothModel', c_ubyte),
    ]



class SpectrumCalibrationType(Structure):
    """SpectrumCalibrationType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_Smoothing', SmoothingType),
        ('m_CalInttime', c_float),
        ('m_aCalibConvers', c_float * MAX_NR_PIXELS),
    ]



class IrradianceType(Structure):
    """IrradianceType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_IntensityCalib', SpectrumCalibrationType),
        ('m_CalibrationType', c_ubyte),
        ('m_FiberDiameter', c_uint32),
    ]



class SpectrumCorrectionType(Structure):
    """SpectrumCorrectionType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_aSpectrumCorrect', c_float * MAX_NR_PIXELS),
    ]



class TriggerType(Structure):
    """TriggerType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_Mode', c_ubyte),
        ('m_Source', c_ubyte),
        ('m_SourceType', c_ubyte),
    ]



class MeasConfigType(Structure):
    """MeasConfigType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_StartPixel', c_uint16),
        ('m_StopPixel', c_uint16),
        ('m_IntegrationTime', c_float),
        ('m_IntegrationDelay', c_uint32),
        ('m_NrAverages', c_uint32),
        ('m_CorDynDark', DarkCorrectionType),
        ('m_Smoothing', SmoothingType),
        ('m_SaturationDetection', c_ubyte),
        ('m_Trigger', TriggerType),
        ('m_Control', ControlSettingsType)
    ]



class TimeStampType(Structure):
    """TimeStampType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_Date', c_uint16),
        ('m_Time', c_uint16),
    ]



class StandAloneType(Structure):
    """StandAloneType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_Enable', c_bool),
        ('m_Meas', MeasConfigType),
        ('m_Nmsr', c_int16)
    ]



class DynamicStorageType(Structure):
    """DynamicStorageType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_Nmsr', c_int32),
        ('m_Reserved', c_ubyte * 8),
    ]



class TempSensorType(Structure):
    """TempSensorType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_aFit', c_float * NR_TEMP_POL_COEF),
    ]



class TecControlType(Structure):
    """TecControlType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_Enable', c_bool),
        ('m_Setpoint', c_float),
        ('m_aFit', c_float * NR_DAC_POL_COEF),
    ]



class ProcessControlType(Structure):
    """ProcessControlType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_AnalogLow', c_float * 2),
        ('m_AnalogHigh', c_float * 2),
        ('m_DigitalLow', c_float * 10),
        ('m_DigitalHigh', c_float * 10),
    ]



class EthernetSettingsType(Structure):
    """EthernetSettingsType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_IpAddr', c_uint32),
        ('m_NetMask', c_uint32),
        ('m_Gateway', c_uint32),
        ('m_DhcpEnabled', c_ubyte),
        ('m_TcpPort', c_uint16),
        ('m_LinkStatus', c_ubyte),
    ]



class OemDataType(Structure):
    """OemDataType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_data', c_ubyte * OEM_DATA_LEN)
    ]



class HeartbeatRespType(Structure):
    """HeartbeatRespType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_BitMatrix', c_uint32),
        ('m_Reserved', c_uint32)
    ]



class DeviceConfigType(Structure):
    """DeviceConfigType Structure."""
    _pack_ = 1
    _fields_ = [
        ('m_Len', c_uint16),
        ('m_ConfigVersion', c_uint16),
        ('m_aUserFriendlyId', c_char * USER_ID_LEN),
        ('m_Detector', DetectorType),
        ('m_Irradiance', IrradianceType),
        ('m_Reflectance', SpectrumCalibrationType),
        ('m_SpectrumCorrect', SpectrumCorrectionType),
        ('m_StandAlone', StandAloneType),
        ('m_DynamicStorage', DynamicStorageType),
        ('m_aTemperature', TempSensorType * MAX_TEMP_SENSORS),
        ('m_TecControl', TecControlType),
        ('m_ProcessControl', ProcessControlType),
        ('m_EthernetSettings', EthernetSettingsType),
        ('m_aReserved', c_ubyte * SETTINGS_RESERVED_LEN),
        ('m_OemData', OemDataType)
    ]



# if IS_WINDOWS:
#     FUNCTYPE = WINFUNCTYPE
# else:
#     FUNCTYPE = CFUNCTYPE

# MeasureCallback = FUNCTYPE(None, POINTER(c_int32), POINTER(c_int32))
# """Used as a decorator for a callback function when a scan is available."""