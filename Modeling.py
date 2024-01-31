
from PyQt6.QtCore import pyqtSignal,QThread


from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import *

import numpy as np
import pandas as pd
import time

class positionScaling(QThread):
    magData = pyqtSignal(float)

    def __init__(self, ctr):

        super().__init__()
        print("Created model")
        self.ctr = ctr
        self.x = self.ctr["x"]
        self.y = self.ctr["y"]
        self.position = None
        self.root = "./data/One_coil.csv"
        self.m = 3.5/(10*3.3*0.3)*1e-6
        
        self.tracker = None
        self.past = 0

        self.B, self.rows, self.columns = self.load_csv(self.root)
        self.i = 0

    def run(self):
        x_prev = self.x
        y_prev = self.y
        while True:

            self.x = self.ctr["x"]*self.m
            self.y = self.ctr["y"]*self.m

            if (self.x != x_prev):
                self.findPoint(self.x, self.y)

            x_prev = self.x
            y_prev = self.y

            if self.ctr["close"]:
                break
            
            time.sleep(0.2)

        self.ctr["close"] = False

        return 1
        

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
        self.ctr["x"] =x
        self.ctr["y"] =x

        x = x*self.m#*2048/342 #342, 256
        y = y*self.m

        x_idx = np.argmin(np.abs((self.rows-np.round(x,5))))
        y_idx = np.argmin(np.abs((self.columns-np.round(y,5))))
        self.past = self.B[x_idx, y_idx]
        #print("coords: ", x, y, "\nidx", x_idx, y_idx,"\n Mag", self.past, "\n----------------")

    def findPoint(self,x,y):
        x_idx = np.argmin(np.abs((self.rows-np.round(x,5))))
        y_idx = np.argmin(np.abs((self.columns-np.round(y,5))))
        current = self.B[x_idx, y_idx]
        error = self.past - current
        self.past = current
        self.magData.emit(error)