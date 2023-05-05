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
import os
from queue import Queue


from NiDriver import niDevice
from baslerControl import baslerCam
from Modeling import positionScaling

class App(QWidget):

    def __init__(self, args):
        super().__init__()

        self.args = args

        #UI geometry
        self.left = 0; self.top = 0
        self.width = 1500; self.height = 1000

        self.samplingHz = args.buffer_size_cfg/(args.samplingFreq)

        #Flags
        self.liveFlag = False
        self.drawFlag = False
        self.trackFlag = False
        self.feedBackFlag = False
        self.tuneFlag = False
        self.calibFlag = False
        self.MeasFlag = False
        self.snapFlag = False
        self.modelFlag = False

        self.rectangleQue = Queue(maxsize=0)

        #Init driver and signal pipe
        self.driver = niDevice(args)
        self.driver.setData.connect(self.receiveData)

        #Init driver and signal pipe
        self.cam = baslerCam(self.args)
        self.cam.changePixmap.connect(self.setImage)
        self.cam.position.connect(self.receiveTrackData)

        self.model = None

        #Variables for drawing
        self.clicks = 0
        self.boundaryFinal = []
        self.pen = QtGui.QPen()
        self.pen.setWidth(5)
        self.pen.setColor(QtGui.QColor("#EB5160"))

        #cfg GUI
        self.initUI()


    def initUI(self):
        
        self.win = QWidget()
        self.styles = {"color": "#f00", "font-size": "20px"}

        self.win.resize(self.width,self.height)
        
        #Main layout
        self.vlayout = QVBoxLayout()

        #1st row: Buttoms 
        self.hbutton = QHBoxLayout()

        #3rd: Viz
        self.hlabels = QHBoxLayout()

        #4th row: sliders, fields and labels
        self.hlayout = QHBoxLayout()
        self.accessory = QVBoxLayout()
        
        self.cfg_buttons() #cfg buttons
        #2nd row: field path 
        self.textLabel()

        self.cfg_plots() #cfg plots
        self.cfg_image() #set label

        #Add final layout and display
        self.win.setLayout(self.vlayout)
        self.win.show()  

    def cfg_image(self):
        """
        Create label, add accesories and label to layout
        """
        self.label = QLabel(self)
        #self.label.resize(480, 480)
    
        background = np.zeros((1536, 2048))
        h, w = background.shape
        bytesPerLine = 1 * w
        convertToQtFormat = QImage(background,w, h, bytesPerLine, QImage.Format.Format_Grayscale8)
        p = convertToQtFormat.scaled(480, 480) #Qt.AspectRatioMode.KeepAspectRatio
        self.label.setPixmap(QPixmap(QPixmap.fromImage(p)))
        self.label.mousePressEvent = self.getPos

        #Accesories
        self.accessory.addWidget(self.snapbtn)
        self.accessory.addWidget(self.trackerCheck)
        self.accessory.addWidget(self.feedBack)
        self.accessory.addWidget(self.autotune)
        self.accessory.addWidget(self.MgFeedback)
        self.accessory.addWidget(self.PositionFeedback)
        self.accessory.addWidget(self.CurrentValueLabel,QtCore.Qt.AlignmentFlag.AlignTop)
        self.accessory.setSpacing(1)
        

        self.hlabels.addLayout(self.accessory)
        self.hlabels.addWidget(self.label)#QtCore.Qt.AlignmentFlag.AlignCenter

        self.hlabels.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.hlabels.setSpacing(100)
        self.vlayout.addLayout(self.hlabels,Qt.AlignmentFlag.AlignCenter)   

    def cfg_plots(self):
        """
        Create plots and add layout
        """
        #Plots
        self.plotI = pg.PlotWidget()
        self.plotB = pg.PlotWidget()
        self.plotTrack = pg.PlotWidget()

        #Init data for the plots
        self.initData()
 

        #Current Plot
        self.dataLineTarget = self.plotI.plot(x=self.time, y=self.target, pen=pg.mkPen(color="red"), symbol='o', symbolSize=10, row = 0, col = 0, name = "target")
        self.dataLineMeasured = self.plotI.plot(x=self.time, y=self.measured, pen=pg.mkPen(color="blue"), symbol='s', symbolSize=10, row = 0, col = 0, name = "Measured current")
        self.plotI.addLegend()
        self.plotI.setTitle("Feedback and real", color="r", size = "15pt")
        self.plotI.setLabel("left", "Current [A]", **self.styles)
        self.plotI.setLabel("bottom", "Time [s]", **self.styles)
        self.plotI.showGrid(x=True, y=True)
        self.plotI.setXRange(0, 10, padding=0, update = False)
        self.plotI.setYRange(-0.2, 2, padding=0, update = False)

        #Mg field plot
        self.dataLineMeasuredB = self.plotB.plot(x=self.time, y=self.measuredB, pen=pg.mkPen(color="green"), symbol='t', symbolSize=10, row = 0, col = 0, name = "Measured B")
        self.plotB.setTitle("Magnetic Field", color="r", size = "15pt")
        self.plotB.setLabel("left", "B [T]", **self.styles)
        self.plotB.setLabel("bottom", "Time [s]", **self.styles)
        self.plotB.showGrid(x=True, y=True)
        self.plotB.setXRange(0, 10, padding=0, update = False)
        self.plotB.setYRange(-0.2, 1.5, padding=0, update = False)

        #Tracking plot
        self.TrackLine = self.plotTrack.plot(x=self.trackX, y=self.trackY, pen=pg.mkPen(color="green"), symbol='t', symbolSize=10, row = 0, col = 0, name = "Track")
        self.plotTrack.setTitle("Tracker", color="r", size = "15pt")
        self.plotTrack.setLabel("left", "y-coordinate", **self.styles)
        self.plotTrack.setLabel("bottom", "x-coordinate", **self.styles)
        self.plotTrack.showGrid(x=True, y=True)
        self.plotTrack.setXRange(0, 2048, padding=0, update = False)
        self.plotTrack.setYRange(0, 1536, padding=0, update = False)        

        #add to the layout
        self.hlayout.addWidget(self.plotI)  
        self.hlayout.addWidget(self.plotB)
        self.hlayout.addWidget(self.plotTrack)
        self.vlayout.addLayout(self.hlayout)  

    def textLabel(self):
        """
        File save path
        """
        self.textField = QTextEdit(self.args.path)
        self.textField.setFixedSize(self.width,50)
        self.vlayout.addWidget(self.textField)

    def createAndCheckFolder(self,path):
        isExist = os.path.exists(path)
        if not isExist:
            # Create a new directory because it does not exist
            os.makedirs(path)

    def initData(self):
        #Init Data
        self.time = np.zeros(500)
        self.target = np.zeros(500)
        self.measured = np.zeros(500)
        self.measuredB = np.zeros(500)
        self.trackX = np.zeros(50)
        self.trackY = np.zeros(50)

    def cfg_buttons(self):
        """
        Create and connect buttons, sliders, and check box
        """

        #Start measurement
        self.btnStart = QPushButton("Start measurement")
        self.btnStart.pressed.connect(self.start)
        self.btnStart.setStyleSheet("background-color : green")
        self.hbutton.addWidget(self.btnStart)

        #Calibration button and autotune checkbox
        self.CalibBtn = QPushButton("Tune")
        self.CalibBtn.pressed.connect(self.calibrate)
        self.CalibBtn.setStyleSheet("background-color : green")
        self.hbutton.addWidget(self.CalibBtn)

        self.autotune = QCheckBox("Autotune")
        self.autotune.stateChanged.connect(self.checkAutoTune)

        self.MgFeedback = QCheckBox("B - Feedback")
        self.MgFeedback.stateChanged.connect(self.changeFeedBack)

        self.PositionFeedback = QCheckBox("Position - Feedback")
        self.PositionFeedback.stateChanged.connect(self.changePosition)
        
        #Stream camera
        self.streamBtn = QPushButton("Video Stream")
        self.streamBtn.pressed.connect(self.livestream)
        self.streamBtn.setStyleSheet("background-color : green")
        self.hbutton.addWidget(self.streamBtn)

        #Stop measurement or tuning
        self.btnStop = QPushButton("stop")
        self.btnStop.pressed.connect(self.stop)
        self.btnStop.setStyleSheet("background-color : red")
        self.hbutton.addWidget(self.btnStop)

        #Close Gui
        btnShutDown = QPushButton("shutdown")
        btnShutDown.pressed.connect(self.shutDown)
        btnShutDown.setStyleSheet("background-color : red")
        self.hbutton.addWidget(btnShutDown)

        #add buttons to main
        self.vlayout.addLayout(self.hbutton)

        #Snap and display image for tracker
        self.snapbtn = QPushButton("snap")
        self.snapbtn.pressed.connect(self.snapImage)
        self.snapbtn.setStyleSheet("background-color : blue")
        #tracker on/off
        self.trackerCheck = QCheckBox("Track")
        self.trackerCheck.stateChanged.connect(self.checkTrack)
        self.trackerCheck.setCheckState(Qt.CheckState.Checked)

        #Feedback on/off
        self.feedBack = QCheckBox("FeedBack")
        self.feedBack.stateChanged.connect(self.checkFeed)
        self.feedBack.setCheckState(Qt.CheckState.Checked)

        #Resistance slider 
        self.Slider1Layout = QVBoxLayout()
        self.sliderR1 = QSlider(Qt.Orientation.Vertical, self)
        self.sliderR1.setRange(0,100)
        self.sliderR1.setValue(27)
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

        #Current slider for tuning
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

        #Display values
        self.CurrentValueLabel = QLabel("Target Current: {:.2f} A \nSource Current: {:.2f} A\nMg sensor: {:.2f} []".format(0.00,0.00,0.00))
        self.CurrentValueLabel.resize(25,25)

        #Add sliders to 4th row
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

    def checkAutoTune(self,state):
        if state == 2:
            self.calibFlag = True
        else:
            self.calibFlag = False

    def changeFeedBack(self,state):
        if state == 2:
            self.driver.BFeedback = True
        else:
            self.driver.BFeedback = False

    def changePosition(self,state):
        if state == 2:
            self.modelFlag = True
            self.model = positionScaling()
            self.cam.changeModelFlag()
            self.model.addCamera(self.cam)
            self.driver.addModel(self.model)
        else:
            self.cam.changeModelFlag()
            self.modelFlag = False
            self.model = None
            self.driver.addModel(None)

    def livestream(self):
        #Start camera stream
        self.liveFlag = True
        self.streamBtn.setStyleSheet("background-color : white")
        
        self.x = threading.Thread(target=self.cam.livestream)
        self.x.start()

    def snapImage(self):
        #snap image
        self.snapFlag = True
        self.snapbtn.setStyleSheet("background-color : white")
        self.snapThread = threading.Thread(target=self.cam.snapImage)
        self.snapThread.start()

    def updateI(self, value):
        #Update input current
        self.sliderILabel.setText(f'Current value: {value/100}')
        self.driver.currentToWrite = value/100

    def updateR(self,value):
        #Update resistance
        self.sliderR1Label.setText(f'Resistance 1 value: {value/100}')
        self.driver.resistance1 = value/100

    def getPos(self, event):
        """
        After snapping an image, 2 clicks to draw rectangle for tracker
        """
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
            self.drawRectangle(self.label.pixmap())
            self.cam.finalboundaries = self.boundaryFinal
            self.cam.initTracker()
            if self.modelFlag:
                self.model.initMag((self.x2-self.x1)/2,((self.y2-self.y1)/2))
            self.snapbtn.setStyleSheet("background-color : green")
            self.snapFlag = False

    def drawRectangle(self,canvas):
        """
        Draw rectangle to the current label
        This goes wrong place!
        """
        painter = QtGui.QPainter(canvas)
        painter.setPen(self.pen)
        
        if self.rectangleQue.empty() == False:
            
            datas = self.rectangleQue.get()
            self.x1 = datas[0]
            self.x2 = datas[1]
            self.y1 = datas[2]
            self.y2 = datas[3]

        
        width = self.x2 - self.x1
        height = self.y2 - self.y1
        painter.drawRect(int(self.x1),int(self.y1), int(width), int(height))
        #painter.drawRect(0,350, int(width), int(height))
        painter.end()
        self.label.setPixmap(canvas)
        with self.rectangleQue.mutex:
            self.rectangleQue.queue.clear()

    def calibrate(self):
        """
        Start manual and automatic cfg
        """
        self.createAndCheckFolder(self.textField.toPlainText())
        self.driver.root = self.textField.toPlainText()

        self.CalibBtn.setStyleSheet("background-color : white")
        
        if self.calibFlag:
            self.xTune = threading.Thread(target=self.driver.autoTune)
            self.xTune.start()
            self.tuneFlag = True
        else:
            self.xTune = threading.Thread(target=self.driver.tune)
            self.xTune.start()
            self.tuneFlag = True
        
        #self.CalibBtn.setStyleSheet("background-color : green")


    def start(self):
        """
        Start measurement
            -Fetch path
            -start current driver and camera
        """
        self.MeasFlag = True
        self.snapbtn.setStyleSheet("background-color : blue")

        self.createAndCheckFolder(self.textField.toPlainText())
        self.cam.path = self.textField.toPlainText()
        self.driver.root = self.textField.toPlainText()

        self.y = threading.Thread(target=self.driver.start)
        self.y.start()
        
        time.sleep(3)

        self.x = threading.Thread(target=self.cam.recordMeasurement)
        self.x.start()
        
        self.btnStart.setStyleSheet("background-color : white")
        
    def cleanplots(self):
        self.dataLineTarget.setData(self.time, self.target)
        self.dataLineMeasured.setData(self.time, self.measured)
        self.dataLineMeasuredB.setData(self.time, self.measuredB)
        self.TrackLine.setData(self.time, self.measuredB)   
        self.plotB.setXRange(0, 10, padding=0, update = False)
        self.plotB.setYRange(-0.2, 1.5, padding=0, update = False)
        self.plotI.setXRange(0, 10, padding=0, update = False)
        self.plotI.setYRange(-0.2, 2, padding=0, update = False)
        self.plotTrack.setXRange(0, 2048, padding=0, update = False)
        self.plotTrack.setYRange(0, 1536, padding=0, update = False)  

    def stop(self):
        """
        Stop tuning, measurement or camera stream and reset Gui
        """
        if self.tuneFlag:

            self.CalibBtn.setStyleSheet("background-color : green")
            self.btnStop.setStyleSheet("background-color : white")

            #Stop driver
            self.driver.kill()
            self.xTune.join()

            #reset
            self.tuneFlag = False
            self.CurrentValueLabel.setText("Target Current: {:.2f} A \nSource Current: {:.2f} A\nMg sensor: {:.2f} []".format(0,0,0))
            self.btnStop.setStyleSheet("background-color : red")
        elif self.liveFlag:
            #Close streaming camera
            self.cam.kill()
            self.x.join()
            time.sleep(1)
            #reset

            self.streamBtn.setStyleSheet("background-color : green")   
            self.liveFlag = False

        elif self.MeasFlag:
            self.btnStart.setStyleSheet("background-color : green")
            self.btnStop.setStyleSheet("background-color : white")

            #Stop camera and current driver
            self.cam.kill()
            self.driver.kill()

            self.x.join()
            self.y.join()

            if self.modelFlag:
                self.model.initMag(0,0)

            self.MeasFlag = False

        else:
            print("Nothing is running!")
            pass


        time.sleep(5)

        #clean
        self.initData()
        self.cleanplots()
        self.boundaryFinal = [] 
        #reset
        self.CurrentValueLabel.setText("Target Current: {:.2f} A \nSource Current: {:.2f} A\nMg sensor: {:.2f} []".format(0,0,0))
        self.btnStop.setStyleSheet("background-color : red")

        print("Ready roll again!")
    
    def shutDown(self):
        """
        Close all
        """
        self.cam.close()
        self.close()
        print("Shutting down")
        exit(0)

    @pyqtSlot(QImage)
    def setImage(self, image):
        """
        Image signal pipe
        """
        self.receivedFrame = image

        if (self.liveFlag == False) & (self.trackFlag == True) & (self.snapFlag == False):

            self.drawRectangle(QPixmap(QPixmap.fromImage(self.receivedFrame)))
        else:
            self.label.setPixmap(QPixmap(QPixmap.fromImage(self.receivedFrame)))

    @pyqtSlot(object)
    def receiveData(self,data):
        """
        DaQ signal pipe
        """
        self.time = np.roll(self.time,-1); self.time[-1] = data[0]*self.samplingHz
        self.target = np.roll(self.target,-1); self.target[-1] = data[1]
        self.measured = np.roll(self.measured,-1); self.measured[-1] = data[2]
        self.measuredB = np.roll(self.measuredB,-1); self.measuredB[-1] = data[3]

        #Update plots
        self.dataLineTarget.setData(self.time, self.target)
        self.dataLineMeasured.setData(self.time, self.measured)
        self.dataLineMeasuredB.setData(self.time, self.measuredB)

        #Update fields
        self.CurrentValueLabel.setText("Target Current: {:.2f} A \nSource Current: {:.2f} A\nMg sensor: {:.2f} []".format(np.abs(data[1]),np.abs(data[2]),np.abs(data[3])))

        #Update axis
        if data[0]*self.samplingHz>10:
            self.plotI.setXRange(data[0]*self.samplingHz-10, data[0]*self.samplingHz+10, padding=0)
            self.plotB.setXRange(data[0]*self.samplingHz-10, data[0]*self.samplingHz+10, padding=0)

    @pyqtSlot(object)
    def receiveTrackData(self,data):
        """
        Tracker signal pipe 2048, 1536
        """
        
        x2 = data[1] 
        x1 = data[0]
        y2 = 1536 - data[3]
        y1 = 1536 - data[2]
        self.trackX = np.roll(self.trackX,-1)
        self.trackY = np.roll(self.trackY,-1)
        self.trackX[-1] = (x2+x1)/2
        self.trackY[-1] = (y2+y1)/2

        self.TrackLine.setData(self.trackX, self.trackY)
        self.rectangleQue.put(np.array([int(data[0]*480/2048), int(data[1]*480/2048), int(data[2]*480/1536), int(data[3]*480/1536)]))


def pymain(args):
    app = QtWidgets.QApplication(sys.argv)
    w = App(args)
    sys.exit(app.exec())

if __name__ == "__main__":
    pymain()


