#!/usr/bin/env python3

import argparse
import json
import serial
import time
import threading

import tkinter as tk
from tkinter import ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
from matplotlib.figure import Figure
import numpy as np


def json_tree_update(tree, parent, dictionary):
    if not isinstance(dictionary, dict):
        return tree.set(parent, 0, dictionary)
    for key, value in dictionary.items():
        if isinstance(value, list):
            value = {str(i): x for i, x in enumerate(value)}
        if isinstance(value, dict) and len(value) == 1:
            ((child_key, value),) = value.items()
            key += "/" + child_key
        node = parent + "/" + key
        try:
            tree.insert(parent, "end", node, text=key)
        except tk.TclError:
            pass
        json_tree_update(tree, node, value)


class SerialJson:
    def __init__(self, port, baudrate, timespan):
        # store params
        self.port = port
        self.baudrate = baudrate
        self.x_range = np.linspace(0, timespan, round(10 * timespan))
        # initialize data
        self.latest_data = {}
        self.latest_time = 0
        self.start_time = time.monotonic()
        self.timestamps = {}
        self.plot_scales = {}
        self.plot_lines = {}
        # create TK window
        self.root = tk.Tk()
        self.root.title("Live JSON Stream")
        self.root.geometry("1280x720")
        self.root.bind("<Escape>", lambda x: self.root.destroy())
        self.window = tk.PanedWindow(self.root)
        self.window.pack(fill=tk.BOTH, expand=1)
        # create tree view
        self.tree = ttk.Treeview(self.window, columns=(0, 1, 2))
        self.tree.heading("#0", text="Message")
        self.tree.heading("#1", text="Data")
        self.tree.heading("#2", text="Scale")
        self.tree.heading("#3", text="Offset")
        self.tree.column("#0", stretch=True)
        self.tree.column("#1", stretch=True, anchor="center")
        self.tree.column("#2", stretch=False, anchor="center", width=100)
        self.tree.column("#3", stretch=False, anchor="center", width=100)
        self.tree.bind("<Double-1>", lambda x: self.set_cell_value(x))
        self.window.add(self.tree)
        # create plot
        self.fig = Figure()
        self.ax = self.fig.add_subplot()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)
        self.window.add(self.canvas.get_tk_widget())

    def run(self):
        # start serial polling thread
        self.serial_thread = threading.Thread(
            target=self.poll_serial, daemon=True
        ).start()
        # start plot animation
        self.animation = animation.FuncAnimation(
            self.fig, self.plot_update, interval=100
        )
        # start periodic treeview update
        self.tk_tree_update()
        # run app
        self.root.mainloop()

    def set_cell_value(self, event):
        # find selected tree item and column
        item = self.tree.selection()[0]
        column = int(self.tree.identify_column(event.x)[1:]) - 1
        # disable edit for Message and Data fields
        if column <= 0 or self.tree.get_children(item):
            return
        # create textbox at location of click
        entry = tk.Text(self.root, width=10, height=1)
        entry.place(x=event.x - 40, y=event.y - 10)
        entry.insert(tk.END, str(self.tree.set(item, column)))
        # save text to tree item
        def save_edit(event):
            try:
                value = float(entry.get(0.0, "end"))
                assert value != 0
                self.tree.set(item, column, value)
                if column == 1:
                    self.plot_scales[item] = value
            except:
                self.tree.set(item, column, "")
                if column == 1:
                    self.plot_scales.pop(item, None)
            entry.destroy()

        # bind save and exit
        entry.bind("<Return>", save_edit)
        entry.bind("<FocusOut>", lambda x: entry.destroy())
        entry.focus_set()

    def poll_serial(self):
        with serial.Serial(self.port, self.baudrate) as ser:
            while True:
                line_bytes = ser.readline()
                line_string = line_bytes.decode("ascii", "ignore")
                try:
                    # update latest data if parse is sucessful
                    data = json.loads(line_string)
                    self.latest_data.update(data)
                    self.latest_time = time.monotonic()
                    self.timestamps.update({key: self.latest_time for key in data})
                except:
                    # otherwise pass through to console
                    print(line_string, end="")

    def plot_update(self, i):
        # delete lines that are disabled
        for key in list(self.plot_lines):
            if key not in self.plot_scales:
                self.plot_lines[key][0].remove()
                self.ax.legend()
                del self.plot_lines[key]
        y_min = 0
        y_max = 0
        # update each enabled line
        for key, scale in self.plot_scales.items():
            # apply scale factor
            try:
                sample = float(self.tree.set(key, 0)) * scale
            except:
                continue
            # apply offset
            try:
                sample += float(self.tree.set(key, 2))
            except:
                pass
            # create new line of not already exits
            if key not in self.plot_lines:
                zeros = np.zeros(len(self.x_range))
                zeros[-1] = sample
                (line,) = self.ax.plot(self.x_range, zeros, label=key)
                self.ax.legend()
                self.plot_lines[key] = (line, zeros)
            else:
                # otherwise append data
                line, samples = self.plot_lines[key]
                samples[:-1] = samples[1:]
                samples[-1] = sample
                line.set_data(self.x_range, samples)
                y_min = min(y_min, samples.min())
                y_max = max(y_max, samples.max())
        # rescale plot limits
        if y_min != 0 or y_max != 0:
            self.ax.set_ylim(y_min * 1.01, y_max * 1.01)

    def tk_tree_update(self):
        # loop every 5 ms
        self.root.after(5, self.tk_tree_update)
        for key, value in self.latest_data.items():
            # only update on new timestamp
            if self.timestamps[key] != self.latest_time:
                continue
            # insert toplevel item if not already
            try:
                self.tree.insert("", "end", key, text=key, open=True)
            except tk.TclError:
                pass
            # update item recursively
            json_tree_update(self.tree, key, value)
            # set timestape of update in data field
            self.tree.set(key, 0, round(self.latest_time - self.start_time, 6))
        # invalidate old timestamps
        self.latest_time = time.monotonic()


parser = argparse.ArgumentParser(
    description="Display and plot JSON stream over serial."
)
parser.add_argument("-p", "--port", type=str, default="/dev/ttyUSB0")
parser.add_argument("-b", "--baudrate", type=int, default=115200)
parser.add_argument("-t", "--timespan", type=float, default=10)
args = parser.parse_args()
SerialJson(port=args.port, baudrate=args.baudrate, timespan=args.timespan).run()
