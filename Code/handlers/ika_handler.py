import os
import time
import serial

class IKAHandler:
    # handles communication with the IKA stirrer hotplate.
    def __init__(self, serial_port: str = '/dev/ttyACM0', baud_rate: int = 9600) -> None:
        try:    
            self.serial_connection = serial.Serial(serial_port, baud_rate, timeout = 1)
            self.serial_connection.write(b'IN_NAME\r\n')
        except Exception as e:
            print("Connection to hotplate failed. Exception: ", e)
            self.serial_connection = None
            
    def send_command(self, command: str) -> str:
        # sends a command to the IKA hotplate and returns its response.
        if not self.serial_connection:
            print("Not connected")
            raise ConnectionError("Not connected to IKA stirrer.")
    
        full_command = (command + '\r\n').encode("ascii")
        self.serial_connection.write(full_command)
        
        response = self.serial_connection.readline().decode("ascii").strip()
        return response
    
    def close_connection(self) -> None:
        # closes the serial connection to the hotplate.
        if self.serial_connection:
            self.serial_connection.close()
            self.serial_connection = None
            print("Successfully closed connection")