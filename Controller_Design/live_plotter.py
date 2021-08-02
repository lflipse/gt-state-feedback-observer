import multiprocessing as mp
import matplotlib.pyplot as plt
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import time

class LivePlotter(mp.Process):
    def __init__(self, data_channel):

        super().__init__(daemon=True)
        self.data_channel = data_channel
        self.exit = False
        self.data = {
            "time_stamp": [],
            "steering_angle": [],
            "steering_rate": [],
            "torque": [],
        }

        # Lay-out
        title = "Live Plot"
        self.csfont = {'fontname': 'Georgia'}
        self.hfont = {'fontname': 'Georgia'}
        self.tud_blue = "#0066A2"
        self.tud_black = "#000000"
        self.tud_grey = "#808080"
        self.tud_red = "#c3312f"
        self.tud_green = "#00A390"
        self.tud_yellow = "#F1BE3E"

    def run(self):
        while not self.exit:
            # Gather data
            data_available = self.data_channel.poll()
            if data_available is True:
                msg = self.data_channel.recv()
                self.exit = msg["exit"]
                self.update_plot(msg)

        print("quited the plotter")

    def update_plot(self, msg):
        self.data["time_stamp"].append(msg["time_stamp"])
        self.data["steering_angle"].append(msg["steering_angle"])





