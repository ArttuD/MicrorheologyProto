from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtWidgets import (QWidget, QTextEdit, QSlider, QGraphicsScene, QGraphicsView, QWidget,QPushButton,QVBoxLayout,QHBoxLayout,QLabel, QCheckBox)
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
        self.width = 1500
        self.height = 1000
        self.i = 0

        self.liveFlag = False
        self.drawFlag = False
        self.trackFlag = False
        self.feedBackFlag = False


        self.driver = niDevice(args)
        
        self.driver.setData.connect(self.receiveData)
        self.cam = baslerCam(self.args)
        self.cam.changePixmap.connect(self.setImage)
        self.cam.position.connect(self.receiveTrackData)
        self.initUI()
        background = np.zeros((640, 480))
        h, w = background.shape
        bytesPerLine = 1 * w
        convertToQtFormat = QImage(background,w, h, bytesPerLine, QImage.Format.Format_Grayscale8)
        self.receivedFrame = convertToQtFormat.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        self.label.setPixmap(QPixmap(QPixmap.fromImage(self.receivedFrame)))

        self.tuneFlag = False
        self.clicks = 0

        self.boundaryFinal = []

        
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
        self.hlabels = QHBoxLayout()
        self.accessory = QVBoxLayout()

        self.hlabels.setSpacing(0)
        self.hlabels.setContentsMargins(0,0,0,0)
        
        self.cfg_buttons()
        self.textLabel()
        self.cfg_plots()
        self.cfg_image()

        self.win.setLayout(self.vlayout)
        self.win.show()  

    def cfg_image(self):

        self.label = QLabel(self)
        self.label.resize(640, 480)
        self.label.mousePressEvent = self.getPos
        
        self.accessory.addWidget(self.snapbtn)
        self.accessory.addWidget(self.trackerCheck)
        self.accessory.addWidget(self.feedBack)
        self.accessory.setSpacing(2)
        
        self.hlabels.addLayout(self.accessory)
        self.hlabels.addWidget(self.label,QtCore.Qt.AlignmentFlag.AlignCenter)

        self.hlabels.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.hlabels.setSpacing(200)
        self.hlabels.setContentsMargins(0,0,0,0)
        self.vlayout.addLayout(self.hlabels,Qt.AlignmentFlag.AlignCenter)

        #self.vlayout.addWidget(self.view)    

    def cfg_plots(self):
        #Plots
        self.plotI = pg.PlotWidget()
        self.plotB = pg.PlotWidget()
        self.plotTrack = pg.PlotWidget()

        self.initData()

        self.dataLineTarget = self.plotI.plot(x=self.time, y=self.target, pen=pg.mkPen(color="red"), symbol='o', symbolSize=10, row = 0, col = 0, name = "target")
        self.dataLineMeasured = self.plotI.plot(x=self.time, y=self.measured, pen=pg.mkPen(color="blue"), symbol='s', symbolSize=10, row = 0, col = 0, name = "Measured current")
      
        #self.dataLineMeasuredB = self.plotI.plot(x=self.time, y=self.measuredIB, pen=pg.mkPen(color="green"), symbol='t', symbolSize=10, row = 0, col = 0, name = "Measured current from B")
        
        self.plotI.addLegend()
        self.plotI.setTitle("Feedback and real", color="r", size = "15pt")
        self.plotI.setLabel("left", "Current [A]", **self.styles)
        self.plotI.setLabel("bottom", "Time [s]", **self.styles)
        self.plotI.showGrid(x=True, y=True)
        self.plotI.setXRange(0, 100, padding=0, update = False)

        self.dataLineMeasuredB = self.plotB.plot(x=self.time, y=self.measuredB, pen=pg.mkPen(color="green"), symbol='t', symbolSize=10, row = 0, col = 0, name = "Measured B")
        
        self.plotB.setTitle("Magnetic Field", color="r", size = "15pt")
        self.plotB.setLabel("left", "B [T]", **self.styles)
        self.plotB.setLabel("bottom", "Time [s]", **self.styles)
        self.plotB.showGrid(x=True, y=True)
        self.plotB.setXRange(0, 100, padding=0, update = False)

        self.TrackLine = self.plotTrack.plot(x=self.trackX, y=self.trackY, pen=pg.mkPen(color="green"), symbol='t', symbolSize=10, row = 0, col = 0, name = "Track")
        self.plotTrack.setTitle("Tracker", color="r", size = "15pt")
        self.plotTrack.setLabel("left", "y-coordinate", **self.styles)
        self.plotTrack.setLabel("bottom", "x-coordinate", **self.styles)
        self.plotTrack.showGrid(x=True, y=True)
        self.plotTrack.setXRange(0, 2048, padding=0, update = False)
        self.plotTrack.setYRange(0, 1536, padding=0, update = False)        
        
        #self.plotI.resize(self.width/2,50)
        #self.plotB.resize(self.width/2,50)

        self.hlayout.addWidget(self.plotI)  
        self.hlayout.addWidget(self.plotB)
        self.hlayout.addWidget(self.plotTrack)
        self.vlayout.addLayout(self.hlayout)  

    def textLabel(self):
        self.textField = QTextEdit(r'C:\Users\Asentaja\Git\MicrorheologyProto\test',self)
        self.textField.setFixedSize(self.width,50)

        self.vlayout.addWidget(self.textField)


    def initData(self):
        #Init Data
        self.time = np.zeros(50)
        self.target = np.zeros(50)
        self.measured = np.zeros(50)
        self.measuredB = np.zeros(50)
        self.trackX = np.zeros(50)
        self.trackY = np.zeros(50)

    def cfg_buttons(self):
        
        #Buttons
        self.btnStart = QPushButton("start")
        self.btnStart.pressed.connect(self.start)
        self.btnStart.setStyleSheet("background-color : green")
        self.hbutton.addWidget(self.btnStart)

        self.CalibBtn = QPushButton("Tune")
        self.CalibBtn.pressed.connect(self.calibrate)
        self.CalibBtn.setStyleSheet("background-color : green")
        self.hbutton.addWidget(self.CalibBtn)
        
        self.streamBtn = QPushButton("Video Stream")
        self.streamBtn.pressed.connect(self.livestream)
        self.streamBtn.setStyleSheet("background-color : green")
        self.hbutton.addWidget(self.streamBtn)

        self.btnStop = QPushButton("stop")
        self.btnStop.pressed.connect(self.stop)
        self.btnStop.setStyleSheet("background-color : red")
        self.hbutton.addWidget(self.btnStop)

        btnShutDown = QPushButton("shutdown")
        btnShutDown.pressed.connect(self.shutDown)
        btnShutDown.setStyleSheet("background-color : red")
        self.hbutton.addWidget(btnShutDown)
    
        self.vlayout.addLayout(self.hbutton)

        self.snapbtn = QPushButton("snap")
        self.snapbtn.pressed.connect(self.snapImage)
        self.snapbtn.setStyleSheet("background-color : blue")

        self.trackerCheck = QCheckBox("Track")
        self.trackerCheck.stateChanged.connect(self.checkTrack)

        self.feedBack = QCheckBox("FeedBack")
        self.feedBack.stateChanged.connect(self.checkFeed)

        self.Slider1Layout = QVBoxLayout()
        self.sliderR1 = QSlider(Qt.Orientation.Vertical, self)
        self.sliderR1.setRange(0,1000)
        self.sliderR1.setValue(15)
        self.sliderR1.setSingleStep(1)
        self.sliderR1.setPageStep(1)
        self.sliderR1.setTickPosition(QSlider.TickPosition.TicksRight)
        self.sliderR1.setTickInterval(10)

        self.sliderR1.valueChanged.connect(self.updateR)
        self.sliderR1Label = QLabel('', self)
        self.sliderR1Label.setText(f'Resistance 1 Value: {self.sliderR1.value()/100}')
        self.driver.resistance1 = self.sliderR1.value()/100
        self.Slider1Layout.addWidget(self.sliderR1Label)
        self.Slider1Layout.addWidget(self.sliderR1)
        self.Slider1Layout.setSpacing(0)

        self.Slider2Layout = QVBoxLayout()
        self.sliderI = QSlider(Qt.Orientation.Vertical, self)
        self.sliderI.setRange(0,300)
        self.sliderI.setValue(0)
        self.sliderI.setSingleStep(1)
        self.sliderI.setPageStep(10)
        self.sliderI.setTickPosition(QSlider.TickPosition.TicksRight)
        self.sliderI.setTickInterval(50)

        self.sliderI.valueChanged.connect(self.updateI)
        self.sliderILabel = QLabel('', self)
        self.sliderILabel.setText(f'Current Value: {self.sliderI.value()/100}')
        self.driver.currentToWrite = self.sliderI.value()/100
        self.Slider2Layout.addWidget(self.sliderILabel)
        self.Slider2Layout.addWidget(self.sliderI)
        self.Slider2Layout.setSpacing(0)

        self.hlabels.addLayout(self.Slider1Layout)
        self.hlabels.addLayout(self.Slider2Layout)

    def checkFeed(self,state):
        if state == 2:
            self.feedBackFlag = True 
            self.driver.feedBackFlag = True
        else:
            self.feedBackFlag = False
            self.driver.feedBackFlag = False

    def checkTrack(self,state):
        if state == 2:
            self.trackFlag = True
            self.cam.trackFlag = True
        else:
            self.trackFlag = False
            self.cam.trackFlag = False


    def livestream(self):
        if self.liveFlag == False:
            print("starting Camera")
            self.x = threading.Thread(target=self.cam.livestream)
            self.x.start()
            self.streamBtn.setStyleSheet("background-color : red")
            self.liveFlag = True
        else:
            print("closing camera")
            self.cam.changeStopFlag()
            self.x.join()
            self.liveFlag = False
            self.streamBtn.setStyleSheet("background-color : green")

    def snapImage(self):
        self.snapbtn.setStyleSheet("background-color : white")
        self.snapThread = threading.Thread(target=self.cam.snapImage)
        self.snapThread.start()

    def updateI(self, value):
        self.sliderILabel.setText(f'Current value: {value/100}')
        self.driver.currentToWrite = value/100

    def getPos(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            x = event.pos().x()
            y = event.pos().y()
            self.boundaryFinal.append([(x,y)])
            self.clicks += 1
        
        if self.clicks == 2:
            self.clicks = 0
            self.x1 = self.boundaryFinal[0][0][0] 
            self.x2 = self.boundaryFinal[1][0][0] 
            self.y1 = self.boundaryFinal[0][0][1]
            self.y2 = self.boundaryFinal[1][0][1]
            self.drawRectangle()
            self.cam.finalboundaries = self.boundaryFinal
            self.cam.initTracker()
            #self.boundaryFinal = []
            self.snapbtn.setStyleSheet("background-color : blue")

    def drawRectangle(self):
        canvas = self.label.pixmap()
        painter = QtGui.QPainter(canvas)
        pen = QtGui.QPen()
        pen.setWidth(3)
        pen.setColor(QtGui.QColor("#EB5160"))
        painter.setPen(pen)
        width = self.x2 - self.x1
        height = self.y2 - self.y1
        painter.drawRect(self.x1, self.y1, width, height)
        painter.end()
        self.label.setPixmap(canvas)


    def calibrate(self):
        self.CalibBtn.setStyleSheet("background-color : white")
        self.xTune = threading.Thread(target=self.driver.tune)
        self.xTune.start()
        self.tuneFlag = True
        self.CalibBtn.setStyleSheet("background-color : green")
        #self.driver.tuneResistance()
        
    def updateR(self,value):
        self.sliderR1Label.setText(f'Resistance 1 value: {value/100}')
        self.driver.resistance1 = value/100

    def start(self):
        print("saving to", self.textField.toPlainText())
        self.cam.path = self.textField.toPlainText()

        self.y = threading.Thread(target=self.driver.start)
        self.y.start()
        time.sleep(5)
        print("starting Camera")
        self.x = threading.Thread(target=self.cam.recordMeasurement)
        self.x.start()
        self.btnStart.setStyleSheet("background-color : white")
        
    
    def stop(self):
        if self.tuneFlag:
            self.CalibBtn.setStyleSheet("background-color : green")
            self.btnStop.setStyleSheet("background-color : white")
            self.driver.running = False
            print("waiting threads to join")
            self.xTune.join()
            print("Closed current driver succesfully!")
            self.driver = niDevice(self.args)
            self.driver.resistance1 = self.sliderR1.value()/100
            self.driver.currentToWrite = 0
            self.driver.setData.connect(self.receiveData)
            self.initData()

            self.dataLineTarget.setData(self.time, self.target)
            self.dataLineMeasured.setData(self.time, self.measured)
            self.dataLineMeasuredB.setData(self.time, self.measuredB)
            self.TrackLine.setData(self.time, self.measuredB)
            self.plotTrack.setXRange(0, 2048, padding=0, update = False)
            self.plotTrack.setYRange(0, 1536, padding=0, update = False)    

            self.plotI.setXRange(0, 100, padding=0)
            self.plotB.setXRange(0, 100, padding=0)

            self.tuneFlag = False
            self.btnStop.setStyleSheet("background-color : red")            
        else:
            self.btnStart.setStyleSheet("background-color : green")
            self.btnStop.setStyleSheet("background-color : white")
            self.cam.changeStopFlag()
            self.driver.running = False
            print("waiting threads to join")
            self.x.join()
            print("Closed camera succesfully!")
            self.y.join()
            print("Closed current driver succesfully!")
            self.driver = niDevice(self.args)
            self.driver.resistance1 = self.sliderR1.value()/100
            self.driver.currentToWrite = 0
            self.driver.setData.connect(self.receiveData)
            self.initData()
            self.dataLineTarget.setData(self.time, self.target)
            self.dataLineMeasured.setData(self.time, self.measured)
            self.dataLineMeasuredB.setData(self.time, self.measuredB)
            self.plotI.setXRange(0, 100, padding=0)
            self.plotB.setXRange(0, 100, padding=0)
            self.btnStop.setStyleSheet("background-color : red")
    
    def shutDown(self):
        self.cam.close()
        self.close()
        print("Shutting down")
        exit(0)

    @pyqtSlot(QImage)
    def setImage(self, image):
        self.receivedFrame = image
        self.label.setPixmap(QPixmap(QPixmap.fromImage(self.receivedFrame)))
        #if self.trackFlag:
        #    self.drawRectangle()

    @pyqtSlot(object)
    def receiveData(self,data):
        self.time = np.roll(self.time,-1); self.time[-1] = data[0]
        self.target = np.roll(self.target,-1); self.target[-1] = data[1]
        self.measured = np.roll(self.measured,-1); self.measured[-1] = data[2]
        
        self.dataLineTarget.setData(self.time, self.target)
        self.dataLineMeasured.setData(self.time, self.measured)

        self.measuredB = np.roll(self.measuredB,-1); self.measuredB[-1] = data[3]
        self.dataLineMeasuredB.setData(self.time, self.measuredB)
        
        if data[0]>100:
            self.plotI.setXRange(data[0]-100, data[0]+100, padding=0)
            self.plotB.setXRange(data[0]-100, data[0]+100, padding=0)

    @pyqtSlot(object)
    def receiveTrackData(self,data):
        self.x2 = data[1] - int((2048-640)/2)
        self.x1 = data[0] - int((2048-640)/2)
        self.y2 = data[3] - int((1536-480)/2)
        self.y1 = data[2] - int((1536-480)/2)
        self.trackX = np.roll(self.trackX,-1); self.trackX[-1] = int((data[1]-data[0])/2)+data[1]
        self.trackY = np.roll(self.trackY,-1); self.trackY[-1] = int((data[3]-data[2])/2)+data[1]

        self.TrackLine.setData(self.trackX, self.trackY)


def pymain(args):
    app = QtWidgets.QApplication(sys.argv)
    w = App(args)
    #w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    pymain()


