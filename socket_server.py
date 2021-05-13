import socket 
from envs import *
from spectrometer.Spectrometer import Spectrometer
import json 
from errors import *
import time
import threading

class SpectrometerServer(): 
    def __init__(self, ip, port):
        self.spectrometer_ready = False
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.HEADERSIZE = 10
        self.INTEGRATION_TIME = 100
        self.s.bind((ip, port))
        self.s.listen(1)
        self.client_socket = None
        self.spectrometer_status = "initiate"
        self.spectrometer_thread = threading.Thread(target=self.spectrometer_listenner, daemon=True)
        self.spectrometer_thread.start()
        print("The server is listenning...")

    def client_connection(self): 
        while True: 
            try:
                self.client_socket, self.address = self.s.accept()
                self.client_ip, self.client_port = self.address
                self.socket_thrd = threading.Thread(target=self.listen_for_data, daemon=True)

                print(f"Assessing connection from {self.client_ip}:{self.client_port}")

                if self.client_ip == ALLOWED_IP: 
                    print(f"Connected from {self.address}")
                    self.socket_thrd.start()
                else: 
                    raise UnauthorizedConnection  
            except UnauthorizedConnection:
                print("Unauthorized connection")
                self.client_socket.send(bytes("Connection not allowed", "utf-8"))
                self.disconnect()
            

    def listen_for_data(self): 
        try: 
            while True: 
                received_cmd = self.client_socket.recv(1024).decode("utf-8")
                if not received_cmd: 
                    break
                if len(received_cmd) > 0:
                    parsed_res = json.loads(received_cmd)
                    self.execute_cmd(parsed_res)           
       
        except Exception as err: 
            print(f"An error occured trying to receive a message: {err}")
            self.disconnect()
    
    def execute_cmd(self, parsed_res): 
        cmd_key = parsed_res["cmd"]
        print(f"Received CMD - {cmd_key}")
        if cmd_key == "spectrometer_status": 
            if self.spectrometer_ready: 
                self.send_data(json.dumps({
                    "error": False, 
                    "context": "spectrometer_status"
                }))
            else: 
                self.send_data(json.dumps({
                    "error": True, 
                    "error_msg": "The spectrometer is not ready yet!", 
                    "error_name": "SPECTROMETER_NOT_READY",
                    "error_code": "-1000"
                }))
        elif cmd_key == "get_spectrometer_ready_status": 
            self.send_data(json.dumps({
                "error": False,
                "context": "get_spectrometer_ready_status",
                "status": self.spectrometer_ready
            }))  
        elif cmd_key == "perform_measurement": 
            self.INTEGRATION_TIME = parsed_res["it"]
            self.NUM_MEASUREMENTS = parsed_res['num_measurements']
            self.spectrometer_status = "measurement"
        elif cmd_key == "png": 
            self.send_data(json.dumps({
                "error": False, 
                "context": "ping"
            }))
        elif cmd_key == "disconnect": 
            self.disconnect()

    def spectrometer_listenner(self): 
        while True:
            if self.spectrometer_status == "initiate":
                self.initiate_spectrometer()        
                self.spectrometer_status  = "listenning"
            elif self.spectrometer_status == "measurement": 
                print("Perform measurement")
                self.perform_measurement(self.NUM_MEASUREMENTS, it=self.INTEGRATION_TIME)
                self.spectrometer_status  = "listenning"
            elif self.spectrometer_status == "listenning": 
                try: 
                    self.equipment.init_spectrometer()
                    if not self.spectrometer_ready: 
                        print("Equipment connected...")
                        self.initiate_spectrometer()  
                        self.spectrometer_status = "initiate"
                except SpectrometerException as spec_error:
                    self.spectrometer_exception(spec_error=spec_error)
            time.sleep(1)

    def initiate_spectrometer(self): 
        try: 
            self.equipment = Spectrometer(DLL_PATH)
            self.equipment.setup_equipment()
            self.spectrometer_ready = True
            if self.client_socket: 
                print("Sending data to client")
                self.send_data(json.dumps({
                    "error": False, 
                    "context": "spectrometer_status"
                }))
        except SpectrometerException as spec_error:
            self.spectrometer_exception(spec_error=spec_error,retry=True)

    def spectrometer_exception(self,spec_error, retry = False): 
        self.spectrometer_ready = False
        error_name, error_msg, error_code = spec_error.args
        print(f"{error_code} - {error_name}: {error_msg}")
        if self.client_socket: 
            self.send_data(json.dumps({
                "error": True, 
                "error_msg": error_msg, 
                "error_name": error_name,
                "error_code": error_code
            }))             
        if retry: 
            time.sleep(1)
            self.initiate_spectrometer()

    def perform_measurement(self, num_measurements, it=100): 
        error, wl, spectra = self.equipment.perform_measurement(num_measurements=num_measurements)
        if error: 
            self.send_data(json.dumps({
                "error": True, 
                "error_msg": "The spectrometer is not ready yet!", 
                "error_name": "SPECTROMETER_NOT_READY",
                "error_code": "-1000"
            }))
        else: 
            self.send_data( json.dumps({
                "error": False, 
                "context": "measurement",
                "spectra": spectra,
                "wl": wl
            }))

    def send_data(self, data): 
        if self.client_socket.fileno() != -1:
            send_data = f'{len(data):<{self.HEADERSIZE}}' + data
            self.client_socket.send(bytes(send_data, "utf-8")) 

    def disconnect(self): 
        print("Client disconnected")
        self.client_socket.close()
        self.client_socket = None

    def disconnect_equipment(self): 
        self.equipment.disconnect()

if __name__ == "__main__":
    server = SpectrometerServer(IPV4_ADD,PORT)
    server.client_connection()
    server.disconnect_equipment()