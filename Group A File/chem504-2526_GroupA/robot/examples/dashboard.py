import sys
import os
import time
import argparse
import math
import socket

#https://s3-eu-west-1.amazonaws.com/ur-support-site/42728/DashboardServer_e-Series_2022.pdf
HOST = "192.168.0.2"
PORT = 29999

def main():
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((HOST, PORT))
    tcp_socket.send(str.encode("power on"))
    tcp_socket.send(str.encode("brake release"))
    #tcp_socket.send(str.encode("power off"))

    
if __name__ == '__main__':
     main()