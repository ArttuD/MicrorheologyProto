from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtWidgets import (QWidget, QGraphicsScene, QGraphicsView, QWidget,QPushButton,QVBoxLayout,QHBoxLayout,QLabel)
from PyQt6.QtGui import QImage, QPixmap

from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys
import numpy as np
import cv2
import time
import threading

from NiDriver import niDevice
from baslerControl import baslerCam

class App(QWidget):

    def __init__(self, args):
        super().__init__()
        
        self.args = args
        self.left = 0; self.top = 0
        self.width = 2000
        self.height = 1500
        self.i = 0
        self.initUI()
        self.data = None

        self.driver = niDevice(args)
        
        self.driver.setData.connect(self.receiveData)

        self.cam = baslerCam(self.args)
        self.cam.changePixmap.connect(self.setImage)
        background = np.zeros((640, 480))
        h, w = background.shape
        bytesPerLine = 1 * w
        convertToQtFormat = QImage(background,w, h, bytesPerLine, QImage.Format.Format_Mono)
        self.receivedFrame = convertToQtFormat.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        self.label.setPixmap(QPixmap.fromImage(self.receivedFrame))

        
    def initUI(self):
        self.setGeometry(self.left, self.top, self.width, self.height)

        self.win = QWidget()
        self.styles = {"color": "#f00", "font-size": "20px"}
        #Make it pleasent
        #self.setBackground("black")
        self.win.resize(self.width,self.height)
        self.vlayout = QVBoxLayout()
        self.vlayout.setSpacing(0)
        self.vlayout.setContentsMargins(0,0,0,0)
        self.hlayout = QHBoxLayout()
        self.hbutton = QHBoxLayout()
        
        self.cfg_buttons()
        self.cfg_plots()
        self.cfg_image()

        self.win.setLayout(self.vlayout)
        self.win.show()  

    def cfg_image(self):
        #Image
        #self.scene = QGraphicsScene()#0, 0, 400, 200
        #self.pixmap = QPixmap() #"//home.org.aalto.fi/lehtona6/data/Documents/data/newplot.png"
        #self.pixmap = self.pixmap.scaled(600, 400, Qt.AspectRatioMode.KeepAspectRatio)
        #self.scene.addPixmap(self.pixmap)
        #self.view = QGraphicsView(self.scene)
        
        self.label = QLabel(self)
        #self.label.move(50, 75)
        self.label.resize(640, 480)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.vlayout.addWidget(self.label)
        #self.vlayout.addWidget(self.view)    

    def cfg_plots(self):
        #Plots
        self.plotI = pg.PlotWidget()
        self.plotB = pg.PlotWidget()

        #Init Data
        self.time = np.zeros(100)
        self.target = np.zeros(100)
        self.measured = np.zeros(100)
        self.measuredB = np.zeros(100)

        self.dataLineTarget = self.plotI.plot(x=self.time, y=self.target, pen=pg.mkPen(color="red"), symbol='o', symbolSize=10, row = 0, col = 0, name = "target")
        self.dataLineMeasured = self.plotI.plot(x=self.time, y=self.measured, pen=pg.mkPen(color="blue"), symbol='s', symbolSize=10, row = 0, col = 0, name = "Measured current")
        self.dataLineMeasuredB = self.plotI.plot(x=self.time, y=self.measuredB, pen=pg.mkPen(color="green"), symbol='t', symbolSize=10, row = 0, col = 0, name = "Measured current from B")
        
        self.plotI.addLegend()
        self.plotI.setTitle("Feedback and real", color="r", size = "15pt")
        self.plotI.setLabel("left", "Current [A]", **self.styles)
        self.plotI.setLabel("bottom", "Time [s]", **self.styles)
        self.plotI.showGrid(x=True, y=True)
        self.plotI.setXRange(0, 10, padding=0, update = False)
        
        self.plotB.setTitle("Magnetic Field", color="r", size = "15pt")
        self.plotB.setLabel("left", "B [T]", **self.styles)
        self.plotB.setLabel("bottom", "Time [s]", **self.styles)
        self.plotB.showGrid(x=True, y=True)
        self.plotI.resize(self.width/2,50)
        self.plotB.resize(self.width/2,50)

        self.hlayout.addWidget(self.plotI)  
        self.hlayout.addWidget(self.plotB)
        self.vlayout.addLayout(self.hlayout)  

        
    def cfg_buttons(self):
        #Buttons
        self.btnStart = QPushButton("start")
        self.btnStart.pressed.connect(self.start)
        self.btnStart.setStyleSheet("background-color : green")
        self.hbutton.addWidget(self.btnStart)
        

        self.btnStop = QPushButton("stop")
        self.btnStop.pressed.connect(self.stop)
        self.btnStop.setStyleSheet("background-color : red")
        self.hbutton.addWidget(self.btnStop)

        btnShutDown = QPushButton("shutdown")
        btnShutDown.pressed.connect(self.shutDown)
        btnShutDown.setStyleSheet("background-color : red")
        self.hbutton.addWidget(btnShutDown)
        self.vlayout.addLayout(self.hbutton)        

    #def setTimer():
        #self.timer = QtCore.QTimer()
        #self.timer.setInterval(100)
        #self.timer.timeout.connect(self.update_plot_data)
        #self.timer.start()

    def start(self):
        #Start coils
        #start measurement
        #self.running = self.driver.start()
        self.x = threading.Thread(target=self.cam.livestream)
        self.x.start()
        self.y = threading.Thread(target=self.driver.start)
        self.y.start()
        self.btnStart.setStyleSheet("background-color : white")
        
    
    def stop(self):
        self.btnStart.setStyleSheet("background-color : green")
        self.btnStop.setStyleSheet("background-color : white")
        self.cam.changeStopFlag()
        self.driver.running = False
        print("waiting threads to join")
        self.x.join()
        print("Closed camera succesfully!")
        self.y.join()
        print("Closed current driver succesfully!")
        self.btnStop.setStyleSheet("background-color : green")
    
    def shutDown(self):
        self.cam.close()
        self.close()
        print("Shutting down")
        exit(0)

    @pyqtSlot(QImage)
    def setImage(self, image):
        self.receivedFrame = image

    def update_plot_data(self):
        
        self.time = np.roll(self.time,-1); self.time[-1] = self.data[0]
        self.target = np.roll(self.target,-1); self.target[-1] = self.data[1]
        self.measured = np.roll(self.measured,-1); self.measured[-1] = self.data[2]

        self.measuredB = np.roll(self.measuredB,-1); self.measuredB[-1] = self.data[3]
        
        self.dataLineTarget.setData(self.time, self.target)
        self.dataLineMeasured.setData(self.time, self.measured)
        self.dataLineMeasuredB.setData(self.time, self.measuredB)
        if self.data[0]>10:
            self.plotI.setXRange(self.data[0]-10, self.data[0]+10, padding=0)
        #self.view.setPixmap(QPixmap.fromImage(self.p))
        self.label.setPixmap(QPixmap.fromImage(self.receivedFrame))

    #Signaling

    @pyqtSlot(object)
    def receiveData(self,datas):
        """
        if self.i%2 == 0:
            img = cv2.imread("//home.org.aalto.fi/lehtona6/data/Documents/data/newplot.png")
            h, w, ch = img.shape
            bytesPerLine = ch * w
            convertToQtFormat = QImage(img, w, h, bytesPerLine, QImage.Format.Format_RGB888)
            self.p = convertToQtFormat.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        else:
            img = cv2.imread("//home.org.aalto.fi/lehtona6/data/Documents/data/pins.png")
            h, w, ch = img.shape
            bytesPerLine = ch * w
            convertToQtFormat = QImage(img, w, h, bytesPerLine, QImage.Format.Format_RGB888)
            self.p = convertToQtFormat.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        #print(self.i)
        self.i += 1
        """
        self.data = datas
        self.update_plot_data()
        self.data = None


def pymain(args):
    app = QtWidgets.QApplication(sys.argv)
    w = App(args)
    #w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    pymain()


