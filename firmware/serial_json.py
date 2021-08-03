#!/usr/bin/env python3

import argparse
import json
import serial
import threading
import tkinter as tk
from tkinter import ttk


def json_tree_update(tree, parent, dictionary):
    if not isinstance(dictionary, dict):
        return tree.set(parent, 0, dictionary)
    for key, value in dictionary.items():
        if isinstance(value, list):
            value = {str(i): x for i, x in enumerate(value)}
        if isinstance(value, dict) and len(value) == 1:
            ((child_key, value),) = value.items()
            key += "_" + child_key
        node = parent + "/" + key
        try:
            tree.insert(parent, "end", node, text=key)
        except tk.TclError:
            pass
        json_tree_update(tree, node, value)


class SerialJson:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.latest_data = None
        self.root = tk.Tk()
        self.root.title("Live Data")
        self.root.bind("<Escape>", lambda x: self.root.destroy())
        self.tree = ttk.Treeview(self.root, columns=0)
        self.tree.pack(expand=1, fill=tk.BOTH)

    def run(self):
        self.serial_thread = threading.Thread(
            target=self.poll_serial, daemon=True
        ).start()
        self.gui_update()
        self.root.mainloop()

    def poll_serial(self):
        with serial.Serial(self.port, self.baudrate) as ser:
            while True:
                line_bytes = ser.readline()
                line_string = line_bytes.decode("ascii", "ignore")
                try:
                    self.latest_data = json.loads(line_string)
                except:
                    print(line_string, end="")

    def gui_update(self):
        self.root.after(10, self.gui_update)
        if self.latest_data:
            json_tree_update(self.tree, "", self.latest_data)
            self.latest_data = None


parser = argparse.ArgumentParser(description="print serial json stream")
parser.add_argument("-p", "--port", type=str, default="/dev/ttyUSB0")
parser.add_argument("-b", "--baudrate", type=int, default=115200)
args = parser.parse_args()
SerialJson(port=args.port, baudrate=args.baudrate).run()
