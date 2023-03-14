from pypylon import pylon
from pypylon import genicam
import numpy as np
import sys
from PyQt6.QtCore import Qt, pyqtSignal,QThread
from PyQt6.QtGui import QImage
import os
import matplotlib.pyplot as plt
import cv2
import cv2
from tracker import tracker

class baslerCam(QThread):
    changePixmap = pyqtSignal(QImage)
    position = pyqtSignal(object)

    def __init__(self, args):
        super().__init__()
        self.path = args.path
        self.frame = None
        self.timeStamp = np.array([0])
        self.stopFlag = False
        self.trackFlag = False
        self.finalboundaries = None
        self.trackerTool = None

        self.cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        print("Camera connected successfully")

        
        self.boundaryFinal = None
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
                #image = grab
                self.frame = grab.GetArray().astype("uint8")
                #self.timeStamp = np.concatenate((self.timeStamp,grab.GetTimeStamp))
                self.emitFrame()
                grab.Release()
            else:
                self.cam.StopGrabbing()
                self.stopFlag = False
                break
        return

    def snapImage(self):
        img_sum = np.zeros((self.height, self.width), dtype=np.uint8)
        with self.cam.GrabOne(1000) as res:
            img = res.Array
        
        img_sum += img
        self.frame = img_sum
        self.emitFrame()

    def initTracker(self):
        self.trackerTool = tracker(self.finalboundaries)
        print("tracker created!")

    def recordMeasurement(self):
        self.cam.StartGrabbing(self.frameCount,pylon.GrabStrategy_LatestImages)
        i = 0
        while self.cam.IsGrabbing():
            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)

            if (grab.GrabSucceeded()) & (self.stopFlag == False):

                #self.timeStamp = np.concatenate((self.timeStamp,grab.GetTimeStamp))
                #image = self.converter.Convert(grab)
                self.frame = grab.GetArray().astype("uint8")
                if self.trackFlag:
                    self.emitPosition(self.trackerTool.main(self.frame))
                if i%5 == 0:
                    self.emitFrame()
                grab.Release()
            else:
                self.cam.StopGrabbing()
                self.stopFlag = False
                break

            i += 1

        np.save(os.path.join(self.path,"FrameInfo_{}.npy".format(self.frameCount)),self.timeStamp)

    def emitPosition(self,pos):
        self.position.emit(pos)

    def emitFrame(self):
        h, w = self.frame.shape
        ch = 1                
        bytesPerLine = ch * w
        #cv2.imshow('graycsale image',self.frame)
        #k = cv2.waitKey(1)
    
        convertToQtFormat = QImage(self.frame, w, h, bytesPerLine, QImage.Format.Format_Grayscale8) #
        self.p = convertToQtFormat.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
        self.changePixmap.emit(self.p)

    def saver(self):
        # save dictionary to person_data.pkl file
        with open('person_data.pkl', 'wb') as fp:
            pickle.dump(person, fp)
            print('dictionary saved successfully to file')

    def close(self):
        #cv2.destroyAllWindows()
        self.cam.Close() 

#if __name__ == "__main__":

    