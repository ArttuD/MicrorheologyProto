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
        self.cam.ExposureTime.SetValue(self.args.exposure*10e3)
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
        self.frameCount = self.args.frameCount

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
                empty = self.killCommand.get()
                self.cam.StopGrabbing()
                self.cam.resetBuffer()
                break
        return

    def snapImage(self):
        """
        Snap one image
        """

        with self.cam.GrabOne(1000) as res:
            self.frame = res.Array.astype("uint8")
        
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
            self.frame = grab.GetArray().astype("uint8")
            if (grab.GrabSucceeded()):
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
        self.cam.StartGrabbing(self.frameCount,pylon.GrabStrategy_LatestImages)
        i = 0
        while self.cam.IsGrabbing():

            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)
            if (grab.GrabSucceeded()) & (self.killCommand.empty() == True):
                self.frame = grab.GetArray().astype("uint8")
                if self.trackFlag:
                    #Send to tracker
                    coords, self.frame = self.trackerTool.main(self.frame)
                    
                    #Emit coordinates
                    self.emitPosition(coords)
                #Viz every 5th
                if i%5 == 0:
                    self.emitFrame()
                grab.Release()
                i += 1
            else:
                results = self.killCommand.get()
                self.cam.StopGrabbing()
                self.finishBuffer(i)
                break


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
        p = convertToQtFormat.scaled(480, 480, Qt.AspectRatioMode.KeepAspectRatio)
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

    