from pypylon import pylon
from pypylon import genicam
import numpy as np
import sys
from PyQt6.QtCore import Qt, pyqtSignal,QThread
from PyQt6.QtGui import QImage
import os
import matplotlib.pyplot as plt
import cv2

class baslerCam(QThread):
    changePixmap = pyqtSignal(QImage)

    def __init__(self, args):
        super().__init__()
        self.path = args.path
        self.frame = None
        self.timeStamp = np.array([0])
        self.stopFlag = False
        #try:
        self.cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        print("Camera connected successfully")
        #except:
        #    print("Failed to connect, please check that your camera is connected!")
        #    sys.exit(0)
        

        self.cam.Open()     
        self.cam.ExposureTime.SetValue(args.exposure*10e3)
        self.cam.AcquisitionFrameRateEnable.SetValue(True)
        self.cam.AcquisitionFrameRate.SetValue(args.framerate)        
        self.cam.PixelFormat.SetValue("Mono8")

        self.exposureTime = self.cam.ExposureTime.GetValue()
        self.width = self.cam.Width.GetValue()
        self.height = self.cam.Height.GetValue()
        self.gain = self.cam.Gain.GetValue()
        self.pixelFormat = self.cam.PixelFormat.GetValue()
        self.FrameRate = self.cam.AcquisitionFrameRate.GetValue()
        self.cam.MaxNumBuffer.SetValue(1000)
        self.frameCount = args.frameCount

        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def setRoi(self,width,height):
        self.width = width
        self.height = height
        self.cam.Width.SetValue(self.width)
        self.cam.Height.SetValue(self.height)


    def changeStopFlag(self):
        self.stopFlag = True

    def livestream(self):
        #self.cam.MaxNumBuffer.SetValue(10)
        self.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        while self.cam.IsGrabbing():
            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)
            if (grab.GrabSucceeded()) & (self.stopFlag == False):
                #image = self.converter.Convert(grab)
                image = grab
                self.frame = image.GetArray()
                #self.timeStamp = np.concatenate((self.timeStamp,grab.GetTimeStamp))
                self.emitFrame()
            else:
                self.cam.StopGrabbing()
                self.stopFlag = False
                break
        return

    def recordMeasurement(self):
        
        self.cam.StartGrabbing(self.frameCount,pylon.GrabStrategy_LatestImages)

        while self.IsGrabbing():
            grab = self.cam.RetrieveResult(2000)

            if (grab.GrabSucceeded()) & (self.stopFlag == False):

                self.timeStamp = np.concatenate((self.timeStamp,grab.GetTimeStamp))
                
                self.frame = grab.GetArray()
                self.emitFrame()
                grab.Release()
            else:
                self.cam.StopGrabbing()
                self.stopFlag = False
                break
        
        np.save(os.path.join(self.path,"FrameInfo_{}.npy".format(self.frameCount)),self.timeStamp)

    def emitFrame(self):
        
        h, w = self.frame.shape
        ch = 1                
        bytesPerLine = ch * w
        convertToQtFormat = QImage(self.frame, w, h, bytesPerLine, QImage.Format.Format_Mono)
        self.p = convertToQtFormat.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        self.changePixmap.emit(self.p)

    def close(self):
        self.cam.Close() 

#if __name__ == "__main__":

    