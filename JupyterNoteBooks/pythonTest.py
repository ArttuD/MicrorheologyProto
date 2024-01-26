import multiprocessing as mp

from PyQt6.QtCore import Qt, pyqtSignal,QThread, pyqtSlot as Slot
from PyQt6.QtGui import QImage


from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtCore import Qt, pyqtSlot, pyqtSignal

from PyQt6.QtWidgets import *
from PyQt6.QtGui import QImage, QPixmap


import sys
import numpy as np
import pandas as pd
import cv2
import time
import threading
import os
import datetime
from queue import Queue
import ffmpeg

class positionScaling(QThread):
    magData = pyqtSignal(float)

    def __init__(self, ctr):

        super().__init__()

        self.ctr = ctr
        self.x = self.ctr["x"]
        self.y = self.ctr["y"]
        self.position = None
        self.root = "./data/One_coil.csv"
        self.m = 3.5/(10*3.3*0.3)*1e-6
        #self.emaFilter = EMA(0.85)
        
        self.tracker = None
        self.past = 0

        self.B, self.rows, self.columns = self.load_csv(self.root)


        self.i = 0

    def run(self):
        print("model")
        x_prev = self.x
        y_prev = self.y
        while True:

            self.x = self.ctr["x"]
            self.y = self.ctr["y"]

            if (self.x != x_prev) :
                print("new updates")
                print("model emiting")
                self.magData.emit(1.0)

            x_prev = self.x
            y_prev = self.y

            if self.ctr["closing"]:
                break

            
            time.sleep(0.5)

        print("model done")
        self.ctr["closing"] = False
        

    def load_csv(self, root):
        
        df = pd.read_csv(root, sep = ",", skiprows= [0,1], names = ["x", "y", "By", "Bx"])
        #drop not needed
        df.drop(df[df["y"].values < (0.0035 - 1536*self.m)].index, inplace = True)
        df.drop(df[df["x"].values > 0.00054].index, inplace = True)
        df["y"] -= df["y"].min()
        B = np.array(np.sqrt(df["By"].values**2 + df["Bx"].values**2))
        
        rows = df["x"].unique()
        columns = df["y"].unique()

        #print("rows", np.min(rows), np.max(rows))
        #print("columns", np.min(columns), np.max(columns))
        n_rows = len(rows)
        n_columns = len(columns)
        B = B.reshape(n_rows, n_columns)

        B = np.hstack((B[:,::-1], B))

        return B, rows, columns

    def initMag(self,x,y):

        x = x*self.m*2048/342 #342, 256
        y = (y*1536/256)*self.m

        x_idx = np.argmin(np.abs((self.rows-np.round(x,5))))
        y_idx = np.argmin(np.abs((self.columns-np.round(y,5))))
        self.past = self.B[x_idx, y_idx]
        #print("coords: ", x, y, "\nidx", x_idx, y_idx,"\n Mag", self.past, "\n----------------")

    #def addCamera(self,camera):
    #    self.tracker = camera
    #    self.tracker.coordsScaler.connect(self.receiver)

    #@pyqtSlot(object)
    def receiver(self, data):
        """
        Receive coordinates from Camera
        """
        if self.i == 10:
            x2 = data[1]
            x1 = data[0]
            y2 =  data[3]#(1536 - data[3])
            y1 =  data[2]#(1536 - data[2])

            self.findPoint((x1+x2)/2*self.m,(y1+y2)/2*self.m)
            self.i = 0
        else:
            self.i += 1
        

    def findPoint(self,x,y):
        x_idx = np.argmin(np.abs((self.rows-np.round(x,5))))
        y_idx = np.argmin(np.abs((self.columns-np.round(y,5))))
        current = self.B[x_idx, y_idx]
        error = self.past - current
        self.past = current
        #print("coords: ", x, y, "\nidx", x_idx, y_idx,"\n Mag", self.past, "\n----------------")
        self.emitData(error)
    
    @pyqtSlot(float)
    def emitData(self):
        self.magData.emit(1)

    @pyqtSlot(np.ndarray)
    def receiver_coord(self, vlaue):
        print("model received coords")

class sender(QThread):

    coords = pyqtSignal(np.ndarray)
    
    def __init__(self, ctr):
        super().__init__()

        self.ctr = ctr
        print("created thread")

    def run(self):
        print("stated")
        while True:
            print("Sender emiting")
            self.coords.emit(np.array((np.random.normal(0,1),np.random.normal(0,1))))
            time.sleep(0.5)
            if self.ctr["closing"]:
                break
        
        self.ctr["closing"] = False
        print("done sender")
            
    
class App(QWidget):

    coords_signal = pyqtSignal(np.ndarray)

    def __init__(self) -> None:
        super().__init__()

        self.model_ctr = {"closing": False, "x": 0., "y": 0.}
        self.ctr = {"closing": False}

        self.sender = sender(self.ctr)
        self.sender.coords.connect(self.receive)

        self.process = QtCore.QThread()
        self.sender.moveToThread(self.process)
        self.process.started.connect(self.sender.run) 

        self.model = positionScaling(self.model_ctr)
        self.model.magData.connect(self.receive_model)
        self.coords_signal.connect(self.model.receiver_coord)
        
        self.process_model = QtCore.QThread()
        self.model.moveToThread(self.process_model)
        self.process_model.started.connect(self.model.run) 

        self.win = QWidget()
        self.vlayout = QVBoxLayout()
    
        self.win.resize(20,20)

        self.btnStart = QPushButton("Start measurement")
        self.btnStart.pressed.connect(self.run)
        self.btnStart.setStyleSheet("background-color : green")

        self.btnStop = QPushButton("Stop")
        self.btnStop.pressed.connect(self.stop)
        self.btnStop.setStyleSheet("background-color : green")

        self.vlayout.addWidget(self.btnStart)
        self.vlayout.addWidget(self.btnStop)
        self.win.setLayout(self.vlayout)

        self.win.show()

    def run(self):
        self.process.start()
        self.process_model.start()
        print("started method")

    def stop(self):

        self.model_ctr["closing"] = True
        self.ctr["closing"] = True

        print("Waiting")
        while((self.ctr["closing"]) | (self.model_ctr["closing"])):
            time.sleep(0.5)

        self.process.terminate()
        self.process_model.terminate()
        self.process.wait()
        self.process_model.wait()
        

        print("Ready")
    

    @pyqtSlot(np.ndarray)
    def receive(self, coords):
        print("received stuff 1")
        self.model_ctr["x"] = coords[0]
        self.model_ctr["y"] = coords[1]

    @pyqtSlot(float)
    def receive_model(self, value):
        print("received stuff model")
        #self.q.put(np.stack((img, img, img), axis=-1))


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    w = App()
    sys.exit(app.exec())




"""
def camera_saving(event_saver, q, path, width, height, ending):

    print("in creating saver")
    
    out_name = os.path.join(path,'measurement_{}_{}.mp4'.format(ending, datetime.date.today()))
    out_process = ( 
    ffmpeg 
    .input('pipe:', format='rawvideo', pix_fmt='rgb24', s='{}x{}'
    .format(width, height)) 
    .output(out_name, pix_fmt='yuv420p') .overwrite_output() 
    .run_async(pipe_stdin=True) 
    )
    idx = 0
    while True:
        if (q.empty() == False):
            packet = q.get()
            packet = (packet/ 255).astype(np.uint8)
            frame = np.stack((packet,packet,packet), axis = -1)
            out_process.stdin.write( frame.tobytes() )
            idx += 1
            print("saving", idx)
        elif idx == 10:
            break
        else:
            if not event_saver.is_set():
                #self.print_str.emit("waiting")
                time.sleep(0.01)
            else:
                #self.print_str.emit("closing")
                break
    
    print("closing stream")
    out_process.stdin.close()
    out_process.wait()

"""