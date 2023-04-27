from pypylon import pylon
from pypylon import genicam

from PyQt6.QtCore import Qt, pyqtSignal,QThread
from PyQt6.QtGui import QImage

import numpy as np
import sys
import os
import matplotlib.pyplot as plt
import cv2

from tracker import tracker
from queue import Queue


class baslerCam(QThread):
    changePixmap = pyqtSignal(QImage)
    position = pyqtSignal(object)

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.path = args.path

        #flags
        self.killCommand = Queue(maxsize=1)
        self.trackFlag = False

        #variables
        self.frame = None
        self.finalboundaries = None
        self.boundaryFinal = None
        self.trackerTool = None
        self.timeStamp = np.array([0])

        #init camera
        self.cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        print("Camera connected successfully")
        self.converter = pylon.ImageFormatConverter()

        # converting to opencv bgr format
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        self.cam.Open()   

        #cfg camera
        self.cfgCamera()

    def kill(self):
        """
        Kill threads
        """
        self.killCommand.put(1)

    def cfgCamera(self):
        """
        Set imaging parameters
        """
        self.cam.ExposureTime.SetValue(self.args.exposure*10e2)
        self.cam.AcquisitionFrameRateEnable.SetValue(True)
        self.cam.AcquisitionFrameRate.SetValue(self.args.framerate)        
        self.cam.PixelFormat.SetValue("Mono8")

        self.exposureTime = self.cam.ExposureTime.GetValue()
        self.width = self.cam.Width.GetValue()
        self.height = self.cam.Height.GetValue()
        self.gain = self.cam.Gain.GetValue()
        self.pixelFormat = self.cam.PixelFormat.GetValue()
        self.FrameRate = self.cam.AcquisitionFrameRate.GetValue()
        self.cam.MaxNumBuffer.SetValue(1000)
        self.frameCount = int(self.args.time*self.args.framerate)

        print(self.frameCount, "rate", self.FrameRate)

        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        
    def setROI(self,width,height):
        self.width = width
        self.height = height
        self.cam.Width.SetValue(self.width)
        self.cam.Height.SetValue(self.height)

    def livestream(self):
        """
        Stream video until kill command
        """
        self.killCommand = Queue(maxsize=1)
        self.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) #return the last frame
        while self.cam.IsGrabbing():
            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)
            if (grab.GrabSucceeded()) & (self.killCommand.empty() == True):
                self.frame = grab.GetArray().astype("uint8")
                # emit frame
                self.emitFrame()
                # release buffer
                grab.Release()
            else:
                grab.Release()
                empty = self.killCommand.get()
                self.cam.StopGrabbing()
                #self.cam.resetBuffer()
                break
        
        print("Camera task done!")
        return 0

    def snapImage(self):
        """
        Snap one image
        """
        self.killCommand = Queue(maxsize=1)
        with self.cam.GrabOne(1000) as res:
            image = self.converter.Convert(res)
            self.frame = image.GetArray()
        
        #Emit frame
        self.emitFrame()

    def initTracker(self):
        """
        Init tracker
        """
        self.trackerTool = tracker(self.finalboundaries)

    def finishBuffer(self,i):
        """
        Processes the remaining frames in the buffer
        """
        while True:
            print("processing the remaining image stream!")
            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)
            if (grab.GrabSucceeded()):
                image = self.converter.Convert(grab)
                self.frame = image.GetArray()
                if self.trackFlag:
                    #Send to tracker
                    coords, self.frame = self.trackerTool.main(self.frame)
                    #Emit coordinates
                    self.emitPosition(coords)
                if i%5 == 0:
                    self.emitFrame()
                
                i +=1
            else:
                print("Camera buffer empty, closing!!")
                break



    def recordMeasurement(self):
        """
        Record predefined number of frames
        """
        #Timed snaps and put to que
        self.cam.StartGrabbingMax(self.frameCount)
        #self.cam.StartGrabbing(pylon.GrabStrategy_LatestImages)
        self.killCommand = Queue(maxsize=1)
        i = 0
        
        while self.cam.IsGrabbing():
            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)
            
            if (grab.GrabSucceeded()) & (self.killCommand.empty() == True) & (i <= self.frameCount):
                image = self.converter.Convert(grab)
                self.frame = image.GetArray()
                #self.frame = grab.GetArray().astype("uint8")
                if self.trackFlag:
                    #Send to tracker
                    coords, self.frame = self.trackerTool.main(self.frame)
                    
                    #Emit coordinates
                    self.emitPosition(coords)
                    pass
                #Viz every 5th
                if i%5 == 0:
                    self.emitFrame()
                grab.Release()
                i += 1
            else:
                grab.Release()
                if self.killCommand.empty() == False:
                    results = self.killCommand.get()
                
                self.cam.StopGrabbing()
                    #self.finishBuffer(i)
        
        print("Camera task done!")
        np.save(os.path.join(self.path,"FrameInfo_{}.npy".format(self.frameCount)),self.timeStamp)

    def emitPosition(self,pos):
        """
        Emits coordinates
        """
        self.position.emit(pos)

    def emitFrame(self):
        """
        Size the frame and send to Qt
        """
        h, w = self.frame.shape
        ch = 1                
        bytesPerLine = ch * w
    
        convertToQtFormat = QImage(self.frame, w, h, bytesPerLine, QImage.Format.Format_Grayscale8) #
        p = convertToQtFormat.scaled(480, 480)#Qt.AspectRatioMode.KeepAspectRatio
        self.changePixmap.emit(p)

    def saver(self):
        """
        save results
        """
        # save dictionary to person_data.pkl file
        with open('person_data.pkl', 'wb') as fp:
            pickle.dump(person, fp)
            print('dictionary saved successfully to file')

    def close(self):
        self.cam.Close() 

#if __name__ == "__main__":

    