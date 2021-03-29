#!/usr/bin/env python

from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.collections as mcoll
import pandas as pd
import numpy as np
from datetime import datetime
import os
import cv2

class serialPlot:
    def __init__(self, serialPort = '/dev/cu.usbserial-14420', serialBaud = 115200, map_dim = [101, 101]):
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
        self.frame_counter = -1
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        self.map_type = ""
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
                if ("[GMAP] Size:" in str_data):
                    print(str_data)
                elif ("MAP-Memory" in str_data) or ("MAP-Centered" in str_data):
                    self.map_type = str_data.split(":")[0]
                    self.startCaptureMap = True
                    self.map_data = []
                    data = str_data.split(",")
                    self.frame_counter = int(data[1])
                    print("> Frame: {}".format(self.frame_counter))
                elif self.startCaptureMap:
                    data_str = str_data.split(',')[0:self.map_dim[0]]
                    data = [int(s) for s in data_str]
                    self.map_data.append(data)
                else:
                    self.map_data = []

                if (len(self.map_data) >= self.map_dim[1]):
                    self.readyToFetch = True
                    self.startCaptureMap = False
                
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
    get_files = lambda DIR: [ os.path.join(DIR, f) for f in os.listdir(DIR) if f.endswith(".png") ]
    # log folder
    def create_folder(DIR, clean=False):
        if not os.path.exists(DIR):
            os.mkdir(DIR)
        elif clean:
            filelist = get_files(DIR)
            for f in filelist:
                os.remove(f)
    create_folder(DIR="log", clean=True)

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
    fig = plt.figure(figsize=(10,10))
    ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
    ax.set_title('Global Map')
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    data2d = np.zeros(map_dim)
    im = ax.imshow(data2d, cmap='terrain', vmin = -50, vmax = 120)
    cbar = fig.colorbar(im, ax=ax)
    
    def animate(i):
        if (s.readyToFetch):
            im.set_data(s.map_data)
            s.readyToFetch = False
            ax.set_title('Global Map ({}) [{}]'.format(s.map_type, s.frame_counter))
            plt.savefig("log/gmap_{}.png".format(s.frame_counter), bbox_inches='tight')
            
        # return a list of the artists that need to be redrawn
        return [im]

    anim = animation.FuncAnimation(
        fig, animate, interval=pltInterval, blit=True, repeat=True)

    plt.show()
    s.close()

    ######## convert log images to video: Upon Closing ###
    fps = 5
    frame_array = []
    pathOut = "log/gmap_{}.mp4".format(datetime.now().strftime("%Y-%m-%dT-%H-%M-%SZ"))
    filelist = get_files(DIR="log")
    filelist.sort(key = lambda x: int(x.split('_')[1].split('.')[0]))
    size = ()
    if len(filelist):
        for f in filelist:
            #reading each files
            print(f)
            img = cv2.imread(f)
            height, width, layers = img.shape
            size = (width,height)
            #inserting the frames into an image array
            frame_array.append(img)
        
        out = cv2.VideoWriter(pathOut,cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
        for i in range(len(frame_array)):
            # writing to a image array
            out.write(frame_array[i])
        out.release()

if __name__ == '__main__':
    main()