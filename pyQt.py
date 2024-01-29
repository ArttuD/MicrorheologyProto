from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtWidgets import *
from PyQt6.QtGui import QImage, QPixmap

import pyqtgraph as pg
import sys
import numpy as np
import multiprocessing as mp

import argparse

import configparser
import logging
import os

import time
from queue import Queue

from NiDriver import niDevice
from baslerControl import baslerCam
from Modeling import positionScaling
from tools.tools import camera_saving



class App(QWidget):

    

    def __init__(self, args, logs):
        super().__init__()

        self.args = args
        self.logs = logs

        #Events
        self.event_cam = mp.Event()
        self.event_NI = mp.Event()

        self.event_saver = mp.Event()
        self.q = mp.Queue()


        #control_dicts
        self.cam_ctr = {"close": False, "mode": 0, "save": False, "track": False}
        self.ni_ctr = {"close": False, "mode": 0, "Bcontrol": False,"res": 0.26, "cur": 0, "save": False, "scaler": 1, "feedback": False}
        self.model_ctr = {"closing": False, "x": 0., "y": 0.}
        self.ctr = {"ni" : False, "camera" : False, "save" : False, "model" : False, "draw": False}
        
        #UI geometry
        self.left = 0; self.top = 0
        self.width = 900; self.height = 900
        self.imgCounter = 0

        self.samplingHz = args.buffer_size_cfg/(args.samplingFreq)

        #Init driver and signal pipe
        self.driver = niDevice(self.args, self.ni_ctr)

        self.driver.setData.connect(self.receiveData) #signals
        self.driver.print_str.connect(self.receive_NI_str) #signals

        self.ni_process = QtCore.QThread()
        self.driver.moveToThread(self.ni_process)
        self.ni_process.started.connect(self.driver.run)

        #Init driver and signal pipe
        self.cam = baslerCam(self.args, self.cam_ctr)
        self.cam.changePixmap.connect(self.setImage)
        self.cam.position.connect(self.receiveTrackData) #signals
        self.cam.print_str.connect(self.receive_cam_str) #signals

        self.cam_process = QtCore.QThread()
        self.cam.moveToThread(self.cam_process)
        self.cam_process.started.connect(self.cam.run)

        #Connect model
        self.model = positionScaling(self.model_ctr)
        self.model.magData.connect(self.receive_model)
        self.process_model = QtCore.QThread()
        self.model.moveToThread(self.process_model)
        self.process_model.started.connect(self.model.run) 

        #Variables for drawin
        self.rectangleQue = Queue(maxsize=0)
        self.clicks = 0
        self.boundaryFinal = []
        self.pen = QtGui.QPen()
        self.pen.setWidth(5)
        self.pen.setColor(QtGui.QColor("#EB5160")) 

        self.model = None

        #cfg GUI
        self.initUI()
        


    def initUI(self):
        
        self.win = QWidget()
        self.styles = {"color": "#f00", "font-size": "20px"}

        self.win.resize(self.width,self.height)
        
        self.vlayout = QVBoxLayout() #Main layout
        self.hbutton = QHBoxLayout() #1st row: Buttoms 
        self.htext = QHBoxLayout() #text
        self.hlabels = QHBoxLayout() #3rd: Viz
        self.hlayout = QHBoxLayout() #4th row: sliders, fields and labels
        self.accessory = QVBoxLayout() #4th row: sliders, fields and labels
        
        self.cfg_buttons() #cfg buttons
        self.textLabel() #2nd row: field path 

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
        self.set_black_screen()
        self.label.mousePressEvent = self.getPos

        #Camera
        self.accessory.addWidget(self.snapbtn)
        self.accessory.addWidget(self.trackerCheck)

        #feedback type
        self.accessory.addWidget(self.feedBack)
        self.accessory.addWidget(self.MgFeedback)
        self.accessory.addWidget(self.PositionFeedback)
        self.accessory.addWidget(self.saving_data)

        #tuning
        self.accessory.addWidget(self.autotune)
        self.accessory.addWidget(self.CurrentValueLabel,QtCore.Qt.AlignmentFlag.AlignTop)
        self.accessory.setSpacing(1)

        self.hlabels.addLayout(self.accessory)
        self.hlabels.addWidget(self.label)#QtCore.Qt.AlignmentFlag.AlignCenter

        self.hlabels.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.hlabels.setSpacing(100)
        self.vlayout.addLayout(self.hlabels,Qt.AlignmentFlag.AlignCenter) 


    def set_black_screen(self):

        background = np.zeros((1536, 2048))
        h, w = background.shape
        bytesPerLine = 1 * w
        convertToQtFormat = QImage(background,w, h, bytesPerLine, QImage.Format.Format_Grayscale8)
        p = convertToQtFormat.scaled(342, 256) 
        self.label.setPixmap(QPixmap.fromImage(p)) 

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
        self.plotB.setYRange(-0.01, 0.15, padding=0, update = False)

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

    def initData(self):
        #Init Data
        self.time = np.zeros(500)

        self.target = np.zeros(500)
        self.measured = np.zeros(500)

        self.measuredB = np.zeros(500)
        
        self.trackX = np.zeros(50)
        self.trackY = np.zeros(50)

    def textLabel(self):
        """
        File save path
        """

        self.textField = QTextEdit(self.args.path)
        self.textField.setFixedSize(int(self.width/2),50)

        self.printLabel = QLabel("Print Field")
   
        self.printLabel.setFixedSize(int(self.width/2),50)
        self.printLabel.setStyleSheet("background-color: white")

        self.htext.addWidget(self.textField)
        self.htext.addWidget(self.printLabel)
        self.vlayout.addLayout(self.htext)

    def createAndCheckFolder(self,path):
        isExist = os.path.exists(path)
        if not isExist:
            # Create a new directory because it does not exist
            os.makedirs(path)


    def cfg_buttons(self):
        """
        Create and connect buttons, sliders, and check box
        """
        #Buttons for modes

        self.autotune = QCheckBox("Autotune")
        self.autotune.stateChanged.connect(self.checkAutoTune)

        #tracker on/off
        self.trackerCheck = QCheckBox("Track")
        self.trackerCheck.stateChanged.connect(self.checkTrack)
        #self.trackerCheck.setCheckState(Qt.CheckState.Checked)

        #Feedback on/off
        self.feedBack = QCheckBox("FeedBack")
        self.feedBack.stateChanged.connect(self.current_feedback)
        self.feedBack.setCheckState(Qt.CheckState.Checked)

        self.MgFeedback = QCheckBox("B - Feedback")
        self.MgFeedback.stateChanged.connect(self.B_feedback)

        self.PositionFeedback = QCheckBox("Position - Feedback")
        self.PositionFeedback.stateChanged.connect(self.changePosition)

        self.saving_data = QCheckBox("Save")
        self.saving_data.stateChanged.connect(self.data_saving)
        self.saving_data.setCheckState(Qt.CheckState.Checked)

        #Start measurement
        self.btnStart = QPushButton("Start measurement")
        self.btnStart.pressed.connect(self.start)
        self.btnStart.setStyleSheet("background-color : green")

        self.hbutton.addWidget(self.btnStart)

        #Calibration button and autotune checkbox
        self.CalibBtn = QPushButton("Manual input")
        self.CalibBtn.pressed.connect(self.calibrate)
        self.CalibBtn.setStyleSheet("background-color : green")

        self.hbutton.addWidget(self.CalibBtn)
        
        #Stream camera
        self.streamBtn = QPushButton("Live")
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
        

        #Resistance slider 
        self.Slider1Layout = QVBoxLayout()
        self.sliderR1 = QSlider(Qt.Orientation.Vertical, self)
        self.sliderR1.setRange(0,100)
        self.sliderR1.setValue(int(self.args.FirstResis*100))
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

    def data_saving(self,state):
        if state == 2:
            self.cam_ctr["saving"] = True 
            self.ni_ctr["saving"] = True 
            self.ctr["save"] = True
        else:
            self.cam_ctr["saving"] = False 
            self.ni_ctr["saving"] = False
            self.ctr["save"] = False

    def current_feedback(self,state):
        if state == 2:
            self.ni_ctr["feedBackFlag"] = True
        else:
            self.ni_ctr["feedBackFlag"] = False

    def B_feedback(self,state):
        if state == 2:
            self.ctr["Bcontrol"] = True
            self.ni_ctr["Bcontrol"] = True
        else:
            self.ctr["Bcontrol"] = False
            self.ni_ctr["Bcontrol"] = False

    def checkTrack(self,state):
        if state == 2:
            self.trackFlag = True
            self.cam_ctr["tracking"] = True
        else:
            self.trackFlag = False
            self.cam_ctr["tracking"] = True

    def changePosition(self,state):
        if state == 2:
            self.ctr["model"] = True
            self.model = positionScaling(self.model_ctr)
        else:
            self.model = None

    def checkAutoTune(self,state):
        if state == 2:
            self.calibFlag = True
        else:
            self.calibFlag = False

    def snapImage(self):
        """
        Snap image to initiate tracker
        *** No problems
        """
        self.reset_frames()
        self.snapbtn.setStyleSheet("background-color : white")
        self.logs.info("Snapped Image")

        #snap image
        self.ctr["camera"] = True
        self.cam_ctr["mode"] = 0

        self.cam_process.start()
        time.sleep(0.1)

        self.cam_process.terminate()
        self.cam_process.wait()

        self.snapbtn.setStyleSheet("background-color : green")

    def livestream(self):
        """
        Start camera stream
        *** No problems
        """

        self.logs.info("Starting Live")
        self.streamBtn.setStyleSheet("background-color : white")
        
        #snap image
        self.ctr["camera"] = True
        self.cam_ctr["mode"] = 1
        self.cam_process.start()

        self.streamBtn.setStyleSheet("background-color : green")


    def calibrate(self):
        """
        -Autotune - calibration of magnetic sensor
        -Manual - Kalibrate current sensor to match the input
        ***Problems
        * Autotune, not tested!
        """
        self.CalibBtn.setStyleSheet("background-color : white")
        self.createAndCheckFolder(self.textField.toPlainText())
        self.driver.root = self.textField.toPlainText()

        #snap image
        self.ctr["ni"] = True
        self.ni_ctr["save"] = True
        
        #self.event_Ni = mp.Event()
        #self.current_q = mp.Queue()
        #self.resistance_q = mp.Queue()

        if self.calibFlag:
            self.logs.info("Autotune")
            self.printLabel.setText("Calibrating Hall Sensor to the input current...")
            self.ni_ctr["mode"] = 0            
        else:
            self.logs.info("Manul input")
            self.printLabel.setText("Manual current manipulation")
            self.ni_ctr["mode"] = 1

        self.ni_process.start()

  
    def start(self):
        """
        Start measurement
            -Fetch path
            -start current driver and camera
        """
        self.logs.info("Starting measurements")
        self.printLabel.setText("Measurement started")

        self.ctr["camera"] = True
        self.ctr["ni"] = True

        self.btnStart.setStyleSheet("background-color : white")

        self.createAndCheckFolder(self.textField.toPlainText())

        self.cam.path = self.textField.toPlainText()
        self.driver.root = self.textField.toPlainText()

        self.cam_ctr["mode"] = 1
        self.ni_ctr["mode"] = 2

        if self.ctr["save"]: 
            self.save_thread = mp.Process(target= camera_saving, args=(self.save_event, self.q, self.textField.toPlainText(), 2048, 1536, "main",))
            self.save_thread.start()

        self.ni_process.start()
        self.cam_process.start()

    def updateI(self, value):
        #Update input current
        self.sliderILabel.setText(f'Current value: {value/100}')
        self.ni_ctr["cur"] = value/100

    def updateR(self,value):
        #Update resistance
        self.sliderR1Label.setText(f'Resistance 1 value: {value/100}')
        self.ni_ctr["res"] = value/100

    def reset_frames(self):
        self.set_black_screen()
        self.cam.trackerTool = None
        self.cam.finalboundaries = None
        self.boundaryFinal = []

    def getPos(self, click):
        """
        After snapping an image, 2 clicks to draw rectangle for tracker
        """
        if click.button() == QtCore.Qt.MouseButton.LeftButton:
            x = click.pos().x()
            y = click.pos().y()
            self.boundaryFinal.append([(x,y)])
            self.clicks += 1
        
        if self.clicks == 2:

            self.x1 = self.boundaryFinal[0][0][0] 
            self.x2 = self.boundaryFinal[1][0][0] 
            self.y1 = self.boundaryFinal[0][0][1]
            self.y2 = self.boundaryFinal[1][0][1]
            self.drawRectangle(self.label.pixmap())

            if self.trackFlag:
                self.cam.finalboundaries = self.boundaryFinal
                self.cam.initTracker()

            self.logs.info("Cropped image from {self.boundaryFinal}")
            
            if self.ctr["model"]:
                self.model.initMag(np.abs((self.x2+self.x1)/2),np.abs(((self.y2+self.y1)/2)))

            self.boundaryFinal = []
            self.clicks = 0
            self.snapbtn.setStyleSheet("background-color : blue")
            #self.clicks = 0

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

    def stop(self):
        """
        Stop tuning, measurement or camera stream and reset Gui
        """
        
        self.logs.info("Stopping the previous event")
        self.btnStop.setStyleSheet("background-color : white")

        if self.ctr["ni"] | self.ctr["camera"]:

            if self.ctr["ni"]:
                self.ni_ctr["close"] = True
                while self.ni_ctr["close"]:
                    time.wait(0.1)
                
                self.ni_process.terminate()
                self.ni_process.wait()

            if self.ctr["camera"]:
                self.cam_ctr["close"] = True

                while self.cam_ctr["close"]:
                    time.wait(0.1)
                
                self.cam_process.terminate()
                self.cam_process.wait()

            #stop camera saver
            if self.cam_ctr["saving"]:
                self.save_event.set()
                self.save_thread.join()
                self.save_event.clear()
                self.q.clear()

            if (self.ctr["save"] != False) & (self.ni_ctr["save"] == True):
                self.ni_ctr["save"] = False

            #if self.ctr["model"]:
            #    self.model.initMag(0,0)

        else:
            self.printLabel.setText("Nothing is running!")

        time.sleep(1)
        self.CalibBtn.setStyleSheet("background-color : green")
        self.btnStart.setStyleSheet("background-color : green")
        self.streamBtn.setStyleSheet("background-color : green")

        #clean
        self.initData()
        self.cleanplots()
        self.reset_frames()
        self.imgCounter = 0 

        #reset
        self.CurrentValueLabel.setText("Target Current: {:.2f} A \nSource Current: {:.2f} A\nMg sensor: {:.2f} []".format(0,0,0))
        self.btnStop.setStyleSheet("background-color : red")
        self.printLabel.setText("Ready")
    
    def shutDown(self):
        """
        Close all
        """
        self.logs.info("Closing")
        self.printLabel.setText("Shutting down")
        self.cam.close()
        exit(0)

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

    @pyqtSlot(np.ndarray)
    def setImage(self, image):
        """
        Image signal pipe
        """

        self.imgCounter += 1
        image = (image).astype(np.uint8)

        if self.ctrl["mode"] != 1:
            self.q.put(image)

            # Create a QImage from the normalized image
            if self.imgCounter%5:
                q_image = QImage(image, image.shape[1], image.shape[0], image.shape[1] * 1, QImage.Format.Format_Grayscale8)
                pixmap = QPixmap.fromImage(q_image)
                p = pixmap.scaled(720, 720) 
                self.label.setPixmap(p)
        else:
            q_image = QImage(image, image.shape[1], image.shape[0], image.shape[1] * 1, QImage.Format.Format_Grayscale8)
            pixmap = QPixmap.fromImage(q_image)
            p = pixmap.scaled(720, 720) 
            self.label.setPixmap(p)


        self.receivedFrame = image

        if  (self.trackFlag == True) & (self.cam_ctr["mode"] == 1):
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
        self.CurrentValueLabel.setText("Target Current: {:.2f} A \nSource Current: {:.2f} A\nMg sensor: {:.2f} []".format(np.abs(data[1]),np.abs(data[2]),data[3]))

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
        self.trackY[-1] = (y2+y1)/2 #.scaled(342, 256)

        self.TrackLine.setData(self.trackX, self.trackY)

        self.model_ctr["x"] = self.trackX
        self.model_ctr["y"] = self.trackY

        self.rectangleQue.put(np.array([int(data[0]*342/2048), int(data[1]*342/2048), int(data[2]*256/1536), int(data[3]*256/1536)]))

    @pyqtSlot(str)
    def receive_NI_str(self,data_str):
        """
        Driver Communication channel
        """
        self.printLabel.setText(data_str)
        self.logs.info("NI dirver: {data_str}")

    @pyqtSlot(str)
    def receive_cam_str(self,data_str):
        """
        Driver Communication channel
        """
        self.printLabel.setText(data_str)

        self.logs.info("NI dirver: {data_str}")

    @pyqtSlot(float)
    def receive_model(self, data):
        self.ni_ctr["scaler"] = data
        

"""
def pymain(args, logs):
    app = QtWidgets.QApplication(sys.argv)
    w = App(args, logs)
    sys.exit(app.exec())
"""

if __name__ == "__main__":
    config = configparser.ConfigParser()

    config_path = sys.argv[0]

    # read specified config file?
    if config_path.endswith('.ini'):
        c_path = os.path.join(os.getcwd(),'configs',config_path)
        if os.path.exists(c_path):
            config.read(c_path)
        else:
            raise NotADirectoryError(c_path)
    else:
        # otherwise get the latest
        config.read(os.path.join(os.getcwd(),'configs','default.ini'))


    parser = argparse.ArgumentParser(description='Microrheology test setup.')
    parser.add_argument('--path', '-p', required = False, type = str, default = r'.\test', help='Save path for all the files, creates folder etc')
    parser.add_argument("--user", "-u", required=False, default = "DEFAULT",
                        help = "buffer size for sampling")
    parser.add_argument("--buffer_size_cfg", "-b", required=False, default = None,
                        help = "buffer size for sampling")
    parser.add_argument("--chans_in", "-c", required=False, default = None,
                        help = "Number of sensors for sampling")
    parser.add_argument("--time", "-t", required=False, default =  None,
                        help = "Total time of the measurement")
    parser.add_argument("--exposure", "-e", required=False, default = None,
                        help = "Camera's exposure time in ms")
    parser.add_argument("--framerate", "-f", required=False, default =  None,
                        help = "Cameras framerate isn fps")
    parser.add_argument("--FirstResis", "-r", required=False, default = None,
                        help = "Resistance of current sensor 1")
    parser.add_argument("--samplingFreq", "-sf", required=False, default = None,
                        help = "Sampling frequency")      
    parser.add_argument("--conversionFactor", required=False, default = None,
                        help = "Convert B to i")    
    parser.add_argument('-d', '--debug',
                        help="Print lots of debugging statements",
                        action="store_const", dest="loglevel",
                        const=logging.DEBUG,
                        default=logging.WARNING)
    parser.add_argument('-v', '--verbose',help="Be verbose",
                        action="store_const", dest="loglevel",
                        const=logging.INFO)
    args = parser.parse_args()


    args.buffer_size_cfg = int(config[args.user]["buffer_size_cfg"] )
    args.chans_in = int(config[args.user]["chans_in"] )
    args.time = int(config[args.user]["time"] )
    args.exposure = int(config[args.user]["exposure"]) 
    args.framerate =int( config[args.user]["framerate"]) 
    args.FirstResis = float(config[args.user]["FirstResis"]) 
    args.samplingFreq = int( config[args.user]["samplingFreq"])
    args.conversionFactor = float( config[args.user]["conversionFactor"])

    isExist = os.path.exists(args.path)

    if not isExist:
    # Create a new directory because it does not exist
        os.makedirs(args.path)

    #launch graph
    logging.basicConfig(
                    format='%(levelname)s:%(message)s',
                    level=args.loglevel)

    logging.info(f'Using: {args.user} configuration')

    #pymain(args, logging)

    app = QtWidgets.QApplication(sys.argv)
    w = App(args, logging)
    sys.exit(app.exec())


