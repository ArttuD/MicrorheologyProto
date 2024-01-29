from pypylon import pylon
from pypylon import genicam

from PyQt6.QtCore import Qt, pyqtSignal,QThread
from PyQt6.QtGui import QImage

import numpy as np
import os
import matplotlib.pyplot as plt
import time
import datetime

from tools.tracker import tracker
from queue import Queue

import multiprocessing as mp
from threading import Event, Thread

class baslerCam(QThread, Event):

    changePixmap = pyqtSignal(np.ndarray)
    position = pyqtSignal(object)
    coordsScaler = pyqtSignal(object)

    print_str = pyqtSignal(str) #self.print_str.emit(pos)
 
    def __init__(self, args, ctr):
        super().__init__()
        
        self.args = args
        self.path = args.path

        self.ctr = ctr


        #flags
        self.trackFlag = False

        #variables
        self.frame = None
        self.finalboundaries = None
        self.boundaryFinal = None
        self.trackerTool = None

        self.timeStamp = np.array([0])

        try:
            #init camera
            self.cam = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.converter = pylon.ImageFormatConverter()

            # converting to opencv bgr format
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            self.cam.Open()   

            #cfg camera
            self.cfgCamera()
            self.print_info()
        except:
            self.print_str.emit("No camera available!")

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
        
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        
    def list_properties(self):
        self.properties = "Width {}, height {}, exposure {}, fps {}, gain {}, and frame count {}".format(self.width, self.height ,self.exposureTime ,self.FrameRate,self.gain,self.frameCount)

    def print_info(self):
        self.print_str.emit("Camera set-up\nMono8 colortype \n{} fps".format( self.frameCount, self.FrameRate))

    def setROI(self,width,height):
        self.width = width
        self.height = height
        self.cam.Width.SetValue(self.width)
        self.cam.Height.SetValue(self.height)

    def run(self):
        value = self.ctr["closing"]

        if value == 0:
            self.snapImage()

        elif value == 1:
            self.livestream()

        elif value == 2:
            self.recordMeasurement()

        self.ctr["closing"] = False

        return 1

    def snapImage(self):
        """
        Snap one image
        """
        self.print_str.emit("Image snapped")
        with self.cam.GrabOne(1000) as res:
            image = self.converter.Convert(res)
            self.frame = image.GetArray()
        #Emit frame
        self.emitFrame()

    def livestream(self):
        """
        Stream video until kill command
        """

        self.print_str.emit("Live streaming")
        #self.print_str.emit(self.list_properties())

        self.cam.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) #return the last frame
        while self.cam.IsGrabbing():
            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)
            if (grab.GrabSucceeded()) & (self.ctr["closing"] == False):
                self.frame = grab.GetArray().astype("uint8")
                # emit frame
                self.emitFrame()
                # release buffer
                grab.Release()
            else:
                self.print_str.emit("Camera task done!")
                grab.Release()
                self.cam.StopGrabbing()
                break
        
        return 0


    def recordMeasurement(self):
        """
        Record predefined number of frames
        """
        self.print_str.emit("starting camera")

        #Timed snaps and put to que
        self.cam.StartGrabbingMax(self.frameCount)
        #self.cam.StartGrabbing(pylon.GrabStrategy_LatestImages)
        i = 0
    
        while self.cam.IsGrabbing():
            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)
            
            if (grab.GrabSucceeded()) & (self.event.is_set() == False) & (i <= self.frameCount):
                image = self.converter.Convert(grab)
                self.frame = image.GetArray()
                
                if self.trackFlag:
                    #Send to tracker
                    coords, self.frame = self.trackerTool.main(self.frame)
                    
                    #Emit coordinates
                    self.emitPosition(coords)

                self.emitFrame()

                grab.Release()
                i += 1
                #print("Camera", i)
            else:
                self.cam.StopGrabbing()
                #_ = self.finishBuffer(i)
                grab.Release()

        if self.ctr["save"]:
            np.save(os.path.join(self.path,"FrameInfo_{}.npy".format(datetime.date.today())),self.timeStamp)


        if self.trackFlag:
            self.trackerTool.saveData(self.path)

        self.print_str.emit("Camera task done!")

        return 1

    def initTracker(self):
        """
        Init tracker
        """
        self.trackerTool = tracker(self.finalboundaries)


    def emitPosition(self,pos):
        """
        Emits coordinates
        """
        self.position.emit(pos)

    def emitFrame(self):
        """
        Size the frame and send to Qt
        """
        #h, w = self.frame.shape
        #ch = 1                
        #bytesPerLine = ch * w
    
        #convertToQtFormat = QImage(self.frame, w, h, bytesPerLine, QImage.Format.Format_Grayscale8) #
        #p = convertToQtFormat.scaled(342, 256)#Qt.AspectRatioMode.KeepAspectRatio setPixmap
        self.changePixmap.emit(self.frame)

    def close(self):
        self.cam.Close() 

#if __name__ == "__main__":
        
"""
    def finishBuffer(self,i):
        
        #Processes the remaining frames in the buffer
        

        self.print_str.emit("Finishing the camera buffer")
        while True:
            self.print_str.emit("processing the remaining image stream!")
            grab = self.cam.RetrieveResult(2000,pylon.TimeoutHandling_ThrowException)
            if (grab.GrabSucceeded()):
                image = self.converter.Convert(grab)
                self.frame = image.GetArray()

                if self.trackFlag:
                    #Send to tracker
                    coords, self.frame = self.trackerTool.main(self.frame)
                    #Emit coordinates
                    self.emitPosition(coords)

                if self.saveFlag:
                    self.saving_que.put(self.frame)

                if i%5 == 0:
                    self.emitFrame()
                
                i +=1
            else:
                self.print_str.emit("Camera buffer empty, closing!!")
                break
        
        return 1


    def camera_saving(self, event_saver, q, path):

        out_name = os.path.join(path,'measurement_{}.mp4'.format(datetime.date.today()))

        out_process = ( 
        ffmpeg 
        .input('pipe:', format='rawvideo', pix_fmt='rgb24', s='{}x{}'
        .format(self.width, self.height)) 
        .output(out_name, pix_fmt='yuv420p') .overwrite_output() 
        .run_async(pipe_stdin=True) 
        )
        f = open(os.path.join(path,'frame_info_{}'.format(datetime.date.today())), 'w')
        f.write("index, stamp")
        idx = 0
        while True:
            if (q.empty() == False):
                frame = q.get()
                stamp = frame.timeStamp()
                frame = frame.GetArray()
                
                frame = np.stack((frame.astype("uint8"),frame.astype("uint8"),frame.astype("uint8")), axis = -1)
                out_process.stdin.write( frame.tobytes() )
                f.write(idx, ",", stamp,"\n")
                idx += 1
                #out.write(frame)
            else:
                if not event_saver.is_set():
                    time.sleep(0.01)
                else:
                    break

        
        #out.release()
        f.close()
        out_process.stdin.close()
        out_process.wait()

        return 1
"""

    