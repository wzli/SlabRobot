#!/usr/bin/env python3

import argparse
import json
import serial
import socket


class SerialJson:
    def __init__(self, device, baudrate, port):
        # store params
        self.device = device
        self.baudrate = baudrate
        self.port = port

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        with serial.Serial(self.device, self.baudrate) as ser:
            while True:
                line_bytes = ser.readline()
                line_string = line_bytes.decode("utf-8", "ignore")
                try:
                    # update latest data if parse is sucessful
                    data = json.loads(line_string)
                    sock.sendto(json.dumps(data).encode(), ("localhost", self.port))
                except:
                    # otherwise pass through to console
                    print(line_string, end="")


parser = argparse.ArgumentParser(
    description="Convert serial json stream to UDP stream."
)
parser.add_argument("-d", "--device", type=str, default="/dev/ttyUSB0")
parser.add_argument("-b", "--baudrate", type=int, default=921600)
parser.add_argument("-p", "--port", type=int, default=9870)
args = parser.parse_args()
SerialJson(
    device=args.device,
    baudrate=args.baudrate,
    port=args.port,
).run()
