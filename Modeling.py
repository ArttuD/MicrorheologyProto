import numpy as np
import pandas as pd
from PyQt6.QtCore import pyqtSignal,QThread, pyqtSlot
from tools.tools import EMA

class positionScaling(QThread):
    magData = pyqtSignal(float)

    def __init__(self):

        super().__init__()
        self.position = None
        self.root = "C:/Users/Asentaja/Git/MicrorheologyProto/resource/Data_B.csv"
        self.m = 3.5/(10*3.3*0.3)*1e-6
        self.emaFilter = EMA(0.85)
        
        self.df = None
        self.past = 0
        self.tracker = None

        self.load_csv(self.root)
        self.field = np.stack([self.df["r"].values,self.df["z"].values], axis = 1)

        self.fieldArray =  np.asarray(self.field)
        self.dBzVal = self.df["dBz"].values
        self.dBrVal = self.df["dBr"].values

        self.i = 0
 

    def load_csv(self, root):
        self.df = pd.read_csv(root, sep = "\t", skiprows= [0], names = ["r", "z", "dBz", "dBr"])
        #drop not needed
        self.df.drop(self.df[self.df["r"]>0.8e-3].index, inplace = True)
        #Flip the frame
        self.df["z"] = self.df["z"].max() - self.df["z"]

    

    def addCamera(self,camera):
        self.tracker = camera
        self.tracker.coordsScaler.connect(self.receiver)

    @pyqtSlot(object)
    def receiver(self, data):

        if self.i == 10:
            x2 = data[1] 
            x1 = data[0]
            y2 = 1536 - data[3]
            y1 = 1536 - data[2]
            self.findPoint((x1+x2)/2,(y1+y2)/2)
            self.i = 0
        else:
            self.i += 1


    def closestValue(self, node):
        deltas = self.fieldArray - node
        dist = np.einsum("ij,ij->i", deltas, deltas)
        return np.argmin(dist)

    def initMag(self,x,y):
        place = np.stack([x,y])
        idx = self.closestValue(place)
        self.past =  np.sqrt((self.dBzVal[idx])**2 + (self.dBzVal[idx])**2)

    def findPoint(self,x,y):
        x = self.emaFilter.filterNow(x)*self.m
        y = self.emaFilter.filterNow(y)*self.m

        place = np.stack([x,y])
        idx = self.closestValue(place)
        
        cB  = np.sqrt((self.dBzVal[idx])**2 + (self.dBzVal[idx])**2)
        
        error = cB-self.past
        #print("plavce",cB, self.past)
        self.past = cB
        

        self.emitData(error)
    
    def emitData(self, data):
        #print("Emiting: ", data)
        self.magData.emit(data)


    
