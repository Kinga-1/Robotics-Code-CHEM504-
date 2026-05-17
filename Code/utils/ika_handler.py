import os
import time
import serial

class IKAHandler:
    def __init__(self):
        
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
            device_name = self.getDeviceName()
            print("Connected to stirring plate: " + str(device_name))
            
        except Exception as e:
            print("Connection to stirring plate failed. Exception: ", e)
            self.ser = None
            
    def send_command(self, cmd):
        # sends a command and returns the response.
        if self.ser is None:
            print("Not connected")
            raise ConnectionError("Not connected to stirring plate.")
    
        fullcmd = (cmd + '\r\n').encode("ascii")
        self.ser.write(fullcmd)
        
        r = self.ser.readline().decode("ascii").strip()
        return r
    
    def getDeviceName(self):
        # gets the device name.
        return self.send_command(f'IN_NAME')
        
    def getStirringSpeed(self):
        # gets the current stirring speed.
        return self.send_command(f'IN_PV_4')
        
    def getTemperature(self):
        # gets the current temperature.
        return self.send_command(f'IN_SP_1')
        
    def setTemperature(self, t):
        # sets the target temperature.
        return self.send_command(f'OUT_SP_1 {t}')
        
    def setStirringSpeed(self, s):
        # sets the target stirring speed.
        return self.send_command(f'OUT_SP_4 {s}')
        
    def startHeater(self):
        # starts the heater.
        return self.send_command(f'START_1')
    
    def startStirring(self):
        # starts stirring.
        return self.send_command(f'START_4')
    
    def stopHeater(self):
        # stops the heater.
        return self.send_command(f'STOP_1')
        
    def stopStirring(self):
        # stops stirring.
        return self.send_command(f'STOP_4')
    
    def close(self):
        # closes the serial connection.
        if self.ser:
            self.ser.close()
            self.ser = None
            print("Successfully closed connection")