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
        
        self.tracker = None
        self.past = 0

        self.B, self.rows, self.columns = self.load_csv(self.root)


        self.i = 0
 

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
    
    def emitData(self, data):
        #print("Emiting: ", data)
        self.magData.emit(data)

    
