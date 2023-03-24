
import nidaqmx
from nidaqmx.stream_readers import AnalogMultiChannelReader, AnalogSingleChannelReader
from PyQt6.QtCore import pyqtSignal,QThread

from datetime import datetime
import time
import numpy as np
import os
import threading
from queue import Queue

from tools.tools import *


class niDevice(QThread):
    setData = pyqtSignal(object)

    def __init__(self, args):
        super().__init__()

        #flags
        self.writerFlag = False
        self.readerFlag = False
        self.feedBackFlag = False

        self.killCommand = Queue(maxsize=1) #Change if too much
        self.writeQue = Queue(maxsize=10)
        self.resOneQue = Queue(maxsize=10)

        #parse arguments
        self.totalTime = args.time
        self.root = args.path
        self.resistance1 = args.FirstResis 
        self.buffer_size = round(args.buffer_size_cfg)
        self.samplingFreq = args.samplingFreq
        self.chans_in = args.chans_in

        #Reserve memory
        self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)
        self.que = Queue(maxsize=self.NSamples) #Change if too much
        self.NpyStorage = np.zeros((4,self.NSamples))
        self.bufferIn = np.zeros((self.chans_in, self.buffer_size))
        self.values = np.zeros((self.chans_in,1))

        self.startTime = None
        self.readerStreamIn = None
        self.currentToWrite = 0
        self.MgOffset = 0
        self.plotCoef = 6.94266382e-01

        self.kalman = KalmanF(offset=0.012)

        self.iteration = 0

        self.fileOut = "NiLog.csv"


    def generateStepWave(self):
        """
        Generate step sequence
        """
        self.sequence = np.zeros(self.NSamples)
        self.measured = np.zeros(self.NSamples)
        self.target = np.zeros(self.NSamples)
        self.time = np.zeros(self.NSamples)
        self.sequence[50:450] = 0.5

    def generateSlope(self):
        """
        Generate linear slope for calibration
        """
        self.sequence = np.zeros(self.NSamples)
        self.sequence[:] = np.arange(0,self.NSamples)*0.001*2

    def askUser(self):
        """
        Turn on relay and wait for stop command
        """
        self.NiDWriter.start()
        self.NiDWriter.write(True, 1000)

        while self.killCommand.empty() == True:
            time.sleep(1)
        print("closing driver tasks!")
        time.sleep(2)
        result = self.killCommand.get()

        print("closing, current driver")
        
        self.NiDWriter.write(False, 1000)

        #Wait until other threads are done and close
        while((self.writerFlag==False) & (self.readerFlag==False)):
            print("Waiting tasks to close")
            time.sleep(1)
        
        #Close all current tasks
        self.closeAll()

    def closeAll(self):
        """
        Reset source and stop, close tasks, reset flags
        """
        for i in range(5):
        #Write zeros to reset to source
            self.NiAlWriter.write(0,1000)

        #stop tasks
        self.NiAlReader.stop()
        self.NiAlWriter.stop()
        self.NiDWriter.stop()

        #close taskst
        self.NiAlReader.close()
        self.NiAlWriter.close()
        self.NiDWriter.close()

        #reset flags
        self.writerFlag = False
        self.readerFlag = False
        

    def cfg_AO_writer_task(self):
        #Config analog output channel between -10 and 10V
        self.NiAlWriter.ao_channels.add_ao_voltage_chan("Dev1/ao0","", min_val=- 10.0, max_val=10.0)

    def cfg_DO_writer_task(self):
        #Config digital output
        self.NiDWriter.do_channels.add_do_chan("Dev1/port0/line0","")

    def cfg_AL_reader_task(self):
        """
        Config continous analog input channel
        """
        # Current reading differential 
        self.NiAlReader.ai_channels.add_ai_voltage_chan("Dev1/ai0", terminal_config = nidaqmx.constants.TerminalConfiguration.DIFF)
        # Mg sensor single ended
        self.NiAlReader.ai_channels.add_ai_voltage_chan("Dev1/ai1", terminal_config = nidaqmx.constants.TerminalConfiguration.RSE, min_val = 0, max_val=5)  
        # sampling rate
        self.NiAlReader.timing.cfg_samp_clk_timing(rate=self.samplingFreq, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                                            samps_per_chan=self.buffer_size)
        #Data stream and buffer
        self.readerStreamIn = AnalogMultiChannelReader(self.NiAlReader.in_stream)
        self.NiAlReader.register_every_n_samples_acquired_into_buffer_event(self.buffer_size, self.readingCallback)

    def readingCallback(self, task_idx, event_type, num_samples, callback_data):
        """
        Calc mean and send to process or close when sequence is over
        """
        if (self.killCommand.empty() == True) & (self.iteration < self.NSamples):
            self.readerStreamIn.read_many_sample(self.bufferIn, self.buffer_size, timeout = nidaqmx.constants.WAIT_INFINITELY)
            self.values = np.append(self.values,np.stack((np.mean(self.bufferIn[0,:]),np.mean(self.bufferIn[1,:])), axis = 0).reshape(2,1), axis = 1)
            self.que.put(self.values[:,-1])
            self.iteration += 1
        else:
            print("shutting down")
            self.readerFlag = True
    
        return 0
    
    def calibrateMgSensor(self):
        """
        Find Mg sensor baseline
        """
        Nsamples = 500

        #Create and close task
        calibR = nidaqmx.Task()
        calibR.ai_channels.add_ai_voltage_chan("Dev1/ai1", terminal_config = nidaqmx.constants.TerminalConfiguration.RSE, min_val = 0, max_val=5)  # has to match with chans_in
        datastream = AnalogSingleChannelReader(calibR.in_stream)
        calibR.start()
        dataBuffer = np.zeros(Nsamples)
        print("Calibrating Mg Sensor")
        for i in range(Nsamples):
            dataBuffer[i] = datastream.read_one_sample()
            time.sleep(0.001)
        
        calibR.stop()

        self.MgOffset = np.mean(dataBuffer)

        print("done sampling, Offset: ", self.MgOffset)
        
        dataBuffer = np.empty(1)
        calibR.close()
        calibR = None

    def kill(self):
        """
        Kill threads
        """
        self.killCommand.put(1)

    def changeWritingCurrent(self,value):
        self.writeQue.put(value)

    def changeWritingCurrent(self,value):
        self.resOneQue.put(value)

    def checkLimits(self, data):
        """
        Check that writing value is in the limits
        """
        if data < -10:
            data = -10
        elif data > 10:
            data = 10

        return data

    def fetchQue(self, QueIndex, writeC):
        """
        Fetch from que and send to filter+PID
        """
        measured = self.que.get()
        measured[0] = measured[0]/self.resistance1
        measured[1] = measured[1]-self.MgOffset
        if self.feedBackFlag:
            kalmanOut =  self.kalman.filtering(measured, writeC)
            data = kalmanOut*self.resistance1
        else:
            data = self.resistance1*writeC
        
        return measured,data

    def process(self):
        """
        Pipe measurement
        """

        self.NiAlWriter.start()
        for i in range(10):
            self.NiAlWriter.write(0,1000)
        
        QueIndex = 0
        while True:
            if (self.que.empty() == False) & (QueIndex < self.NSamples):
                #fetch from que
                measured,data = self.fetchQue(QueIndex, self.sequence[QueIndex])
                data = self.checkLimits(data)
                
                #write
                self.NiAlWriter.write(data,1000)

                #Emit Qt
                self.s  = np.array([QueIndex, self.sequence[QueIndex], self.plotCoef*self.sequence[QueIndex] + measured[0], measured[1]])
                self.setData.emit(self.s)

                #Collect
                self.NpyStorage[0,QueIndex] = QueIndex
                self.NpyStorage[1,QueIndex] = self.sequence[QueIndex]
                self.NpyStorage[2,QueIndex] = measured[0]
                self.NpyStorage[3,QueIndex] = measured[1]

                QueIndex += 1
            else:
                #empty que or 
                if (self.killCommand.empty() == True) & (QueIndex < self.NSamples):
                    #sampling has not started yet
                    time.sleep(0.001)
                else:
                    #Empty que and measurement done
                    self.writerFlag = True
                    print("Closing driver")
                    break
        #Save log
        np.save(os.path.join(self.root,"driver.npy"), self.NpyStorage)

    def searchwriter(self):
        if self.writeQue.empty():
            return self.currentToWrite
        else:
            return self.writeQue.get()

    def searchResOne(self):
        if self.resOneQue.empty():
            return self.resistance1
        else:
            return self.resOneQue.get()

    def processTune(self):
        """
        Tuning measurement
        """
        self.NiAlWriter.start()
        for i in range(5):
            self.NiAlWriter.write(0,1000)

        QueIndex = 0
        while True:
            if self.que.empty() == False:
                #fetch data
                self.currentToWrite = self.searchWriter()
                measured,data = self.fetchQue(QueIndex, self.currentToWrite)
                data = self.checkLimits(data)
                #write
                self.NiAlWriter.write(data,1000)

                #Emit to Qt
                self.s  = np.array([self.iteration, self.currentToWrite, self.plotCoef*self.sequence[QueIndex] + measured[0], measured[1]])
                self.setData.emit(self.s)

                QueIndex += 1
            else:
                if self.killCommand.empty() == True:
                    time.sleep(0.001)
                else:
                    self.writerFlag = True
                    break
            

    def processAutoTune(self):
        """
        Pipe autocalibration
        """

        self.NiAlWriter.start()
        for i in range(5):
            self.NiAlWriter.write(0,1000)
        
        QueIndex = 0
        while True:
            if self.que.empty() == False:

                #fetch
                measured,data = self.fetchQue(QueIndex, self.currentToWrite)
                data = self.checkLimits(data)

                #write
                self.NiAlWriter.write(data,1000)

                #emit
                self.s  = np.array([QueIndex, self.currentToWrite, self.plotCoef*self.sequence[QueIndex] + measured[0], measured[1]])
                self.setData.emit(self.s)

                #save
                self.measured[QueIndex] = self.measured[0]
                self.target[QueIndex] = self.target[QueIndex]
                self.time[QueIndex] = QueIndex
                QueIndex += 1
            else:
                if self.killCommand.empty() == True:
                    pass
                    time.sleep(0.001)
                else:
                    self.writerFlag = True
                    break
        """ ToDo derive coef to match plot and source"""

    def initTasks(self):
        """
        Init and cfg tasks
        """
        self.NiAlReader = nidaqmx.Task()
        self.NiAlWriter = nidaqmx.Task()
        self.NiDWriter = nidaqmx.Task()

        self.cfg_AO_writer_task()
        self.cfg_DO_writer_task()
        self.cfg_AL_reader_task()

    def tune(self):
        """
        Manual tune source and ref. resistance
        """
        self.initTasks()

        threadUser = threading.Thread(target = self.askUser)
        threadProcess = threading.Thread(target = self.processTune)
        
        self.start = datetime.now()
        threadUser.start() #turn on voltage source and wait kill command
        
        self.calibrateMgSensor() #calibrate the Mg sensor
        
        threadProcess.start() #start writer
        self.NiAlReader.start() #start reading 

        return 1

    def autoTune(self):
        """
        Manual tune source and ref. resistance with linear sequence
        """
        self.generateSlope()
        self.initTasks()

        threadUser = threading.Thread(target = self.askUser)
        threadProcess = threading.Thread(target = self.processAutoTune)
        
        self.start = datetime.now()
        threadUser.start() #turn on voltage source and wait kill command

        self.calibrateMgSensor() #calibrate the Mg sensor  
        
        threadProcess.start() #start writer
        self.NiAlReader.start() #start reading 

        return 1


    def start(self):
        """
        Measurement
        """

        self.generateStepWave()
        self.initTasks()

        threadUser = threading.Thread(target = self.askUser)
        threadProcess = threading.Thread(target = self.process)

        self.start = datetime.now()
        threadUser.start() #turn on voltage source and wait kill command

        self.calibrateMgSensor() #calibrate the Mg sensor  
        
        threadProcess.start() #start writer
        self.NiAlReader.start() #start reading 

        return 1



