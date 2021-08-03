#!/usr/bin/env python3

import argparse
import json
import serial
import time
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
        self.latest_data = {}
        self.start_time = time.monotonic()
        self.latest_time = 0
        self.timestamps = {}
        self.root = tk.Tk()
        self.root.title("Live Data")
        self.root.geometry("400x800")
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
                    data = json.loads(line_string)
                    self.latest_data.update(data)
                    self.latest_time = time.monotonic()
                    self.timestamps.update({key: self.latest_time for key in data})
                except:
                    print(line_string, end="")

    def gui_update(self):
        self.root.after(5, self.gui_update)
        for key, value in self.latest_data.items():
            if self.timestamps[key] != self.latest_time:
                continue
            try:
                self.tree.insert("", "end", key, text=key, open=True)
            except tk.TclError:
                pass
            self.tree.set(key, 0, round(self.latest_time - self.start_time, 6))
            json_tree_update(self.tree, key, value)
        self.latest_time = time.monotonic()


parser = argparse.ArgumentParser(description="print serial json stream")
parser.add_argument("-p", "--port", type=str, default="/dev/ttyUSB0")
parser.add_argument("-b", "--baudrate", type=int, default=115200)
args = parser.parse_args()
SerialJson(port=args.port, baudrate=args.baudrate).run()
