#!/usr/bin/env python3

import argparse
import socket
import time
import inputs, os, io, fcntl

import ctypes
from libslab.build import libslab_py

libslabfunc = ctypes.cdll.LoadLibrary("libslab/build/libslab.so")


class GamepadUdp:
    def __init__(self, addr, port):
        self.endpoint = (addr, port)
        self.inputs = {}
        self.gamepad = inputs.devices.gamepads[0]
        self.gamepad._character_file = io.open(
            self.gamepad._character_device_path, "rb"
        )
        fd = self.gamepad._character_file.fileno()
        flag = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flag | os.O_NONBLOCK)

    def update_inputs(self):
        if events := self.gamepad._do_iter():
            for event in events:
                self.inputs[event.code] = event.state
        gamepad_msg = libslab_py.GamepadMsg()
        gamepad_msg.buttons = (
            (self.inputs.get("BTN_SELECT", 0) * libslab_py.GAMEPAD_BUTTON_SELECT)
            | (self.inputs.get("BTN_THUMBL", 0) * libslab_py.GAMEPAD_BUTTON_L3)
            | (self.inputs.get("BTN_THUMBR", 0) * libslab_py.GAMEPAD_BUTTON_R3)
            | (self.inputs.get("BTN_START", 0) * libslab_py.GAMEPAD_BUTTON_START)
            | (self.inputs.get("BTN_DPAD_UP", 0) * libslab_py.GAMEPAD_BUTTON_UP)
            | (self.inputs.get("BTN_DPAD_RIGHT", 0) * libslab_py.GAMEPAD_BUTTON_RIGHT)
            | (self.inputs.get("BTN_DPAD_DOWN", 0) * libslab_py.GAMEPAD_BUTTON_DOWN)
            | (self.inputs.get("BTN_DPAD_LEFT", 0) * libslab_py.GAMEPAD_BUTTON_LEFT)
            | (min(self.inputs.get("ABS_Z", 0), 1) * libslab_py.GAMEPAD_BUTTON_L2)
            | (min(self.inputs.get("ABS_RZ", 0), 1) * libslab_py.GAMEPAD_BUTTON_R2)
            | (self.inputs.get("BTN_TL", 0) * libslab_py.GAMEPAD_BUTTON_L1)
            | (self.inputs.get("BTN_TR", 0) * libslab_py.GAMEPAD_BUTTON_R1)
            | (self.inputs.get("BTN_NORTH", 0) * libslab_py.GAMEPAD_BUTTON_TRIANGLE)
            | (self.inputs.get("BTN_EAST", 0) * libslab_py.GAMEPAD_BUTTON_CIRCLE)
            | (self.inputs.get("BTN_SOUTH", 0) * libslab_py.GAMEPAD_BUTTON_CROSS)
            | (self.inputs.get("BTN_WEST", 0) * libslab_py.GAMEPAD_BUTTON_SQUARE)
        )
        gamepad_msg.left_stick.x = self.inputs.get("ABS_X", 128) - 128
        gamepad_msg.left_stick.y = self.inputs.get("ABS_Y", 128) - 128
        gamepad_msg.right_stick.x = self.inputs.get("ABS_RX", 128) - 128
        gamepad_msg.right_stick.y = self.inputs.get("ABS_RY", 128) - 128
        gamepad_msg.left_trigger = self.inputs.get("ABS_Z", 0)
        gamepad_msg.right_trigger = self.inputs.get("ABS_RZ", 0)
        return gamepad_msg

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        buf_len = libslabfunc.GamepadMsg__serialize()
        buf = (ctypes.c_char * buf_len)()
        while True:
            gamepad_msg = self.update_inputs()
            libslabfunc.GamepadMsg__serialize(ctypes.byref(gamepad_msg), buf)
            sock.sendto(buf, self.endpoint)
            time.sleep(0.005)


parser = argparse.ArgumentParser(description="Send gamepad message over UDP")
parser.add_argument("-a", "--address", type=str, default="192.168.4.1")
parser.add_argument("-p", "--port", type=int, default=9871)
args = parser.parse_args()
GamepadUdp(addr=args.address, port=args.port).run()
