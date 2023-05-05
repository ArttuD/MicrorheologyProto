import numpy as np
import pandas as pd
from PyQt6.QtCore import pyqtSignal,QThread, pyqtSlot
from tools.tools import EMA

class positionScaling(QThread):
    magData = pyqtSignal(float)

    def __init__(self):

        super().__init__()
        self.position = None
        self.root = "C:/Users/Asentaja/Git/MicrorheologyProto/resource/gradB.csv"
        self.emaFilter = EMA(0.85)
        
        self.df = pd.read_csv(self.root, sep = "\t", skiprows= [0], names = ["r", "z", "dBz", "dBr"])
        self.df.drop(self.df[self.df["r"]>0.8e-3].index, inplace = True)

        #Flip the frame
        self.df["z"] = self.df["z"].max() - self.df["z"]

        self.field = np.stack([self.df["r"].values,self.df["z"].values], axis = 1)
        self.fieldArray =  np.asarray(self.field)
        self.dBzVal = self.df["dBz"].values
        self.dBrVal = self.df["dBr"].values
        self.past = None

        self.tracker = None

    def addCamera(self,camera):
        self.tracker = camera
        self.tracker.coordsScaler.connect(self.receiver)

    @pyqtSlot(object)
    def receiver(self, data):
        self.findPoint(data[0],data[1])


    def closestValue(self, node):
        deltas = self.fieldArray - node
        dist = np.einsum("ij,ij->i", deltas, deltas)
        return np.argmin(dist)

    def initMag(self,x,y):
        place = np.stack([x,y])
        idx = self.closestValue(place)
        self.past =  (self.dBzVal[idx])**2 + (self.dBzVal[idx])**2

    def findPoint(self,x,y):
        x = self.emaFilter.filterNow(x)
        y = self.emaFilter.filterNow(y)
        place = np.stack([x,y])
        idx = self.closestValue(place)
        cB  = (self.dBzVal[idx])**2 + (self.dBzVal[idx])**2
        
        error = cB-self.past
        #self.past = cB
        self.emitData(error)
    
    def emitData(self, data):
        self.magData.emit(data)


    
