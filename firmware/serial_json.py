#!/usr/bin/env python3

import argparse
import serial
import yaml

parser = argparse.ArgumentParser(description="print serial json stream")
parser.add_argument("-p", "--port", type=str, default="/dev/ttyUSB0")
parser.add_argument("-b", "--baudrate", type=int, default=115200)
args = parser.parse_args()

with serial.Serial(args.port, args.baudrate) as ser:
    while True:
        line_bytes = ser.readline()
        line_string = line_bytes.decode("ascii", "ignore")
        try:
            obj = yaml.safe_load(line_string)
            assert obj
            line_string = yaml.safe_dump(obj, sort_keys=False)
        except:
            pass
        print(line_string, end="")
