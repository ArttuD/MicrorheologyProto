from pypylon import pylon
from pypylon import genicam
import numpy as np
import sys
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QImage
import os

class baslerCam():
    changePixmap = pyqtSignal(QImage)

    def __init__(self, args):
        self.path = args.path
        self.frame = None
        self.timeStamp = np.array([0])
        self.stopFlag = False

        try:
            self.cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            print("Camera connected successfully")
        except:
            print("Failed to connect, please check that your camera is connected!")
            sys.exit(0)
        self.cam.Open()
        
        self.cam.ExposureTime.setValue(args.exposure)
        self.cam.AcquisitionFrameRateEnable.SetValue(True)
        self.cam.AcquisitionFrameRate.setValue(args.framerate)        
        self.cam.PixelFormat.SetValue("MONO8")

        self.exposureTime = self.cam.ExposureTime.GetValue()
        self.width = self.cam.Width.GetValue()
        self.height = self.cam.Height.GetValue()
        self.gain = self.cam.Gain.GetValue()
        self.pixelFormat = self.cam.PixelFormat.GetValue()
        self.FrameRate = self.cam.AcquisitionFrameRate.GetValue()

        self.frameCount = args.frameCount

    def setRoi(self,width,height):
        self.width = width
        self.height = height
        self.cam.Width.setValue(self.width)
        self.cam.Height.setValue(self.height)

    def stopFlag(self):
        self.stopFlag = True

    def livestream(self):
        self.cam.MaxNumBuffer.setValue(10)
        self.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        while self.cam.IsGrabbing():
            grab = self.cam.RetrieveResult(100,pylon.TimeoutHandling_ThrowException)
            if (grab.GrabSucceeded()) & (self.stopFlag == False):
                self.frame = grab.GetArray()
                #self.timeStamp = np.concatenate((self.timeStamp,grab.GetTimeStamp))
                self.emitFrame()
                grab.release()
            else:
                self.cam.StopGrabbing()
                self.stopFlag = False
                break

    def recordMeasurement(self):
        self.cam.setMaxNumBuffer.setValue(1000)
        self.cam.StartGrabbing(self.frameCount,pylon.GrabStrategy_LatestImages)

        while self.IsGrabbing():
            grab = self.cam.RetrieveResult(1000, pylon.TimeoutHandling_ThrowException)

            if (grab.GrabSucceeded()) & (self.stopFlag == False):

                self.timeStamp = np.concatenate((self.timeStamp,grab.GetTimeStamp))
                self.frame = grab.GetArray()
                self.emitFrame()
                grab.release()
            else:
                self.cam.StopGrabbing()
                self.stopFlag = False
                break
        
        np.save(os.path.join(self.path,"FrameInfo_{}.npy".format(self.frameCount)),self.timeStamp)

    def emitFrame(self):
        h, w, ch = self.frame.shape                
        bytesPerLine = ch * w
        convertToQtFormat = QImage(self.frame, w, h, bytesPerLine, QImage.Format.Format_RGB888)
        self.p = convertToQtFormat.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        self.changePixmap.emit(self.p)

    def close(self):
        self.cam.close()   

#if __name__ == "__main__":

    