#!/usr/bin/env python

from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.collections as mcoll
import struct
import pandas as pd
import numpy as np


class serialPlot:
    def __init__(self, serialPort = '/dev/cu.usbserial-14630', serialBaud = 115200, map_dim = [101, 101]):
        self.port = serialPort
        self.baud = serialBaud
        self.map_data = []
        self.rawData = [bytearray(map_dim[0] * 2)]
        self.map_dim = map_dim
        self.data2d = np.zeros(map_dim)
        self.isRun = True
        self.isReceiving = False
        self.readyToFetch = False
        self.startCaptureMap = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        # self.csvData = []

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.rawData = self.serialConnection.readline()
            str_data = self.rawData.decode('utf-8')
            if (self.readyToFetch == False):
                if ("MAP" in str_data):
                    self.startCaptureMap = True
                    self.map_data = []
                elif self.startCaptureMap:
                    data = [int(s) for s in str_data.split(',')]
                    self.map_data.append(data[0:self.map_dim[0]])
            if (len(self.map_data) >= self.map_dim[1]):
                self.readyToFetch = True
                
            # self.serialConnection.readinto(self.map_data)
            self.isReceiving = True

    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')


def main():
    # initializes all required variables
    map_dim_mm = [1000, 1000]
    map_res_mm = 10
    map_dim = list(np.int16(np.array(map_dim_mm)/map_res_mm  + 1))
    print(map_dim)
    s = serialPlot(map_dim=map_dim)
    # starts background thread
    s.readSerialStart()

    # plotting starts below
    pltInterval = 100    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = map_dim[0]
    ymin = 0
    ymax = map_dim[1]
    # plot
    fig = plt.figure()
    ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
    ax.set_title('Global Map')
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    data2d = np.zeros(map_dim)
    im = ax.imshow(data2d, cmap='tab20', vmin = -127, vmax = 127)
    cbar = fig.colorbar(im, ax=ax)
    
    def animate(i):
        if (s.readyToFetch):
            im.set_data(s.map_data)
        # return a list of the artists that need to be redrawn
        return [im]

    anim = animation.FuncAnimation(
        fig, animate, interval=pltInterval, blit=True, repeat=True)
    plt.show()

    s.close()


if __name__ == '__main__':
    main()