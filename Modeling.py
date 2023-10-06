import numpy as np
import pandas as pd
from PyQt6.QtCore import pyqtSignal,QThread, pyqtSlot
from tools.tools import EMA
import scipy

class positionScaling(QThread):
    magData = pyqtSignal(float)

    def __init__(self):

        super().__init__()
        self.position = None
        self.root = "./data/One_coil.csv"
        self.m = 3.5/(10*3.3*0.3)*1e-6

        #self.emaFilter = EMA(0.85)
        
        self.df = None
        self.tracker = None

        self.past = 0
        self.symmetry_point = 5e-5

        self.df = self.load_csv(self.root)

        self.i = 0
 

    def load_csv(self, root):
        """
        Load and preprocess
        1) change axis, easier to remember :)
        2) Reduce dimensions to relevant for FoV and edit move symmetry axis of the core to zero
        3) Round coordinates to 1e-5 accuracy
        4) Smooth FEM steps
        """
        df = pd.read_csv(root, sep = ",", skiprows= [0,1], names = ["x", "y", "By", "Bx"])
        #drop not needed
        df.drop(df[df["y"].values < (3.5-0.9)*1e-3].index, inplace = True)
        df.drop(df[df["x"].values > 0.3*1e-3].index, inplace = True)

        df["y"] -= max(df["y"].values)
        df["y"] = np.abs(df["y"])

        df["x"] = np.round(df["x"],5)
        df["y"] = np.round(df["y"],5)

        df["Bx"] = scipy.ndimage.gaussian_filter1d(df["Bx"].values,5)
        df["By"] = scipy.ndimage.gaussian_filter1d(df["By"].values,5)

        self.x = df["x"].values
        self.y = df["y"].values

        self.Bx = df["Bx"].values
        self.By = df["By"].values


        return df

    def initMag(self,x,y):
        x = x*self.m
        y = y*self.m

        y = np.abs(y - self.symmetry_point)
        mg_idx = np.where((self.x == np.round(x,5)) & (self.y == np.round(y,5)))
        cB  = np.sqrt(self.Bx[mg_idx]**2 + self.By[mg_idx]**2)
        self.past = cB


    def addCamera(self,camera):
        self.tracker = camera
        self.tracker.coordsScaler.connect(self.receiver)

    @pyqtSlot(object)
    def receiver(self, data):
        """
        Receive coordinates from Camera
        """
        if self.i == 10:
            x2 = data[1]
            x1 = data[0]
            y2 = (1536 - data[3])
            y1 = (1536 - data[2])

            self.findPoint((x1+x2)/2*self.m,(y1+y2)/2*self.m)
            self.i = 0
        else:
            self.i += 1
        

    def findPoint(self,x,y):
        y = np.abs(y - self.symmetry_point)

        mg_idx = np.where((self.x == np.round(x,5)) & (self.y == np.round(y,5)))
        cB  = np.sqrt(self.Bx[mg_idx]**2 + self.By[mg_idx]**2)
        error = cB-self.past

        if self.past == 0:
            self.emitData(1)
            self.past = cB
        else:
            self.past = cB
            print("Coords", x,y, "\nError", error)
            self.emitData(error)
    
    def emitData(self, data):
        #print("Emiting: ", data)
        self.magData.emit(data)


    
