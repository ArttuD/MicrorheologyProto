import nidaqmx
from nidaqmx.stream_readers import AnalogMultiChannelReader, AnalogSingleChannelReader
from PyQt6.QtCore import pyqtSignal,QThread, pyqtSlot

import matplotlib.pyplot as plt
import time
import datetime
import numpy as np
import os
#import multiprocessing as mp
#from multiprocessing import Event
from threading import Event, Thread
from queue import Queue

from tools.tools import *



class niDevice(QThread, Event):

    setData = pyqtSignal(object)
    print_str = pyqtSignal(str) #self.print_str.emit


    def __init__(self, event, args):
        super().__init__()
        self.args = args

        self.event = event
        #flags
        self.feedBackFlag = False
        self.saveFlag = False
        self.BFeedback = False
        self.modelFlag = False

        self.writeQue = Queue(maxsize=100)
        self.resOneQue = Queue(maxsize=100)
        self.modelScaler = Queue(maxsize=0)

        #parse arguments
        self.totalTime = args.time
        self.root = args.path
        self.resistance1 = self.args.FirstResis 
        self.buffer_size = round(args.buffer_size_cfg)
        self.samplingFreq = args.samplingFreq
        self.chans_in = args.chans_in

        #Reserve memory
        self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)
        self.FreqRet = int(self.samplingFreq/args.buffer_size_cfg)
        self.que = Queue(maxsize=self.NSamples) #Change if too much
        self.NpyStorage = np.zeros((5,self.NSamples))
        self.bufferIn = np.zeros((self.chans_in, self.buffer_size))
        self.values = np.zeros((self.chans_in,1))

        self.readerStreamIn = None
        self.currentToWrite = 0
        self.cutOFF = None
        
        self.Mgcoef =  15 #Convert 
        self.MgOffset = 2.5

        self.T2i_coef = 0.071565487

        self.model_coef = self.T2i_coef # args.conversionFactor #Convert 
        self.scaler =  0.0
        self.sequence = None
        self.iteration = 0

        self.model = None
        self.properties = None

    def list_properties(self):
        self.properties = "Saving to {}. Running with Hall B slope: {}, offset {}, B2i conversion {}, Resistance {}, and model scaling {}".format( self.root, self.Mgcoef, self.MgOffset, self.T2i_coef, self.resistance1, self.model_coef)

    def addModel(self,model):
        self.modelFlag = True
        self.model = model
        self.model.magData.connect(self.receiveScaler)

    def askUser(self):

        """
        Turn on relay and wait for stop command
        """

        self.NiDWriter.start()
        self.NiDWriter.write(True, 1000)

        while True:
            if self.event.is_set():
                break
            else:
                time.sleep(2)
                    
        self.NiDWriter.write(False, 1000)
        self.NiDWriter.stop()
        
        #wait here the thread!

        while True:
            if self.NiAlReader.is_task_done():
                break
            else:
                time.sleep(1)

        if self.modelFlag:
            with self.modelScaler.mutex:
                self.modelScaler.queue.clear()

        with self.modelScaler.mutex:
            self.modelScaler.queue.clear()

        with self.modelScaler.mutex:
            self.modelScaler.queue.clear()

        self.threadProcess.join()
        self.iteration = 0 
        self.NiAlReader.close()
        self.NiAlWriter.close()
        self.NiDWriter.close()

        return 1


    def updateScaler(self):
        if self.modelScaler.empty() == False:
            self.scaler = self.modelScaler.get()/self.T2i_coef
            #print("scaler:", self.scaler)
    
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
        if (self.event.is_set() == False) & (self.iteration < self.NSamples):
            self.readerStreamIn.read_many_sample(self.bufferIn, self.buffer_size, timeout = nidaqmx.constants.WAIT_INFINITELY)
            self.values = np.append(self.values,np.stack((np.mean(self.bufferIn[0,:]),np.abs(np.mean(self.bufferIn[1,:]))), axis = 0).reshape(2,1), axis = 1)
            self.que.put(self.values[:,-1])
            self.iteration += 1
        else:
            self.NiAlReader.stop()
            
        return 0

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

    def changeWritingCurrent(self,value):
        self.writeQue.put(value)

    def changeWritingRes(self,value):
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
        #print(measured, self.scaler)
        measured[0] = 2*measured[0]
        measured[1] = (measured[1]-self.MgOffset)/self.Mgcoef
        if self.feedBackFlag:
            #print(measured[1]/self.T2i_coef*self.resistance1, measured[1]/self.T2i_coef, measured[1], writeC)
            kalmanOut =  self.kalman.filtering(np.array([measured[0],measured[1]/self.T2i_coef*self.resistance1]), (writeC-self.scaler)*self.resistance1)
            data = kalmanOut
        else:
            data = (writeC-self.scaler)*self.resistance1
        
        return measured,data

    def write_empty(self):

        for i in range(5):
            self.NiAlWriter.write(0,1000)

    def process(self):
        """
        Pipe measurement
        """

        self.NiAlWriter.start()
        self.write_empty()
        
        QueIndex = 0
        
        while True:
            if (self.que.empty() == False) & (QueIndex < self.NSamples):
                
                #fetch from que
                measured, data = self.fetchQue(QueIndex, self.sequence[QueIndex])

                #data =  self.sequence[QueIndex]
                data = self.checkLimits(data)
                
                #Emit Qt
                if QueIndex%10 == 0:
                    self.s  = np.array([QueIndex, self.sequence[QueIndex], measured[0]/self.resistance1, measured[1]]) #self.plotCoef*self.sequence[QueIndex] +
                    #print(self.s)
                    self.setData.emit(self.s)

                if self.modelFlag:
                    self.updateScaler()
            
                #write
                self.NiAlWriter.write(data,1000)

                #Collect
                self.NpyStorage[0,QueIndex] = QueIndex*1/self.FreqRet
                self.NpyStorage[1,QueIndex] = self.sequence[QueIndex]
                self.NpyStorage[2,QueIndex] = measured[0]/self.resistance1
                self.NpyStorage[3,QueIndex] = measured[1]
                self.NpyStorage[4,QueIndex] = self.scaler
                QueIndex += 1
            else:
                #empty que or 
                if (self.event.is_set() == False) & (QueIndex < self.NSamples):
                    #sampling has not started yet
                    time.sleep(0.001)
                else:
                    #Empty que and measurement done
                    break
        #Save log
        self.write_empty()
        self.NiAlWriter.stop()
        if self.saveFlag:
            np.save(os.path.join(self.root,"driver_{}.npy".format(datetime.date.today())), self.NpyStorage)

        return 1


    def processTune(self):

        """
        Tuning measurement
        """

        
        self.NiAlWriter.start()
        self.write_empty()

        QueIndex = 0
        while True:
            if (self.que.empty() == False) & (QueIndex < self.NSamples):
                
                #fetch data
                self.currentToWrite = self.searchwriter()
                self.resistance1 = self.searchResOne()

                measured,data = self.fetchQue(QueIndex, self.currentToWrite)

                data = self.checkLimits(data)
                
                #write
                self.NiAlWriter.write(data,1000)
                
                #Emit to Qt
                if QueIndex%10 == 0:
                    self.s  = np.array([self.iteration, self.currentToWrite, measured[0]/self.resistance1, measured[1]])
                    #print(self.s)
                    self.setData.emit(self.s)

                QueIndex += 1
            else:
                if (self.event.is_set() == False) & (QueIndex < self.NSamples):
                    time.sleep(0.001)
                else:
                    break
        
        self.write_empty()
        self.NiAlWriter.stop()

        return 1

    def processAutoTune(self):
        """
        Pipe autocalibration
        """
        self.NiAlWriter.start()
        self.write_empty()
        
        QueIndex = 0
        while True:
            if (self.que.empty() == False) & (QueIndex < self.NSamples):

                #fetch
                measured,data = self.fetchQue(QueIndex, self.sequence[QueIndex])

                data = self.checkLimits(data)

                #write
                self.NiAlWriter.write(data,1000)

                #emit
                if QueIndex%10 == 0:
                    self.s  = np.array([QueIndex, self.sequence[QueIndex], measured[0]/self.resistance1, measured[1]]) #self.plotCoef*self.sequence[QueIndex] +
                    self.setData.emit(self.s)

                self.NpyStorage[0,QueIndex] = QueIndex*1/self.FreqRet
                self.NpyStorage[1,QueIndex] = self.sequence[QueIndex]
                self.NpyStorage[2,QueIndex] = measured[0]/self.resistance1
                self.NpyStorage[3,QueIndex] = measured[1]
                self.NpyStorage[4,QueIndex] = self.scaler
                QueIndex += 1
            else:
                if (self.event.is_set() == False) & (QueIndex < self.NSamples):
                    time.sleep(0.001)
                else:
                    break

        self.write_empty()
        
        self.NiAlWriter.stop()

        np.save(os.path.join(self.root,"calib_{}.npy".format(datetime.date.today())), self.NpyStorage)

        self.fitCalibration()

        self.totalTime = self.args.time
        self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)

        return 1
            

    def fitCalibration(self):

        maxIndex = np.where(self.NpyStorage[2,:] >=0.5)[0][0]

        x = self.NpyStorage[2,self.cutOFF:maxIndex]
        y = self.NpyStorage[3,self.cutOFF:maxIndex]

        k, b= np.polyfit(x,y,1)
        
        self.print_str.emit("New conversion factor: {}".format(k))
        
        self.T2i_coef = float(k)

        plt.scatter(x,y, label = "data", color = "red", alpha= 0.4)
        plt.plot(x, k*x +b, label = "fit", color = "blue")
        plt.legend()
        plt.savefig(os.path.join(self.root,'calib_{}.png'.format(datetime.date.today())))   # save the figure to file
        plt.close()
        self.print_str.emit("fit results:  k: {}, b: {}".format(k, b))

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
        
        self.print_str.emit(self.list_properties())
        self.kalman = KalmanF(self.BFeedback,offset=0.012, freq= self.FreqRet)
        self.NSamples = np.inf #Continue until stopped
        self.initTasks()

        self.threadProcess =  Thread(target = self.processTune)
        
        #self.#calibrateMgSensor(=() #calibrate the Mg sensor
        
        self.NiAlReader.start() #start reading 
        self.threadProcess.start() #start writer
        
        _ = self.askUser()

        self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)

        return 1


    def start(self):
        """
        Measurement
        """
        self.print_str.emit(self.list_properties())
        
        self.kalman = KalmanF(self.BFeedback,offset=0.012, freq = self.FreqRet)
        self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)

        self.generateStepWave()
        #self.SinWave()
        
        self.initTasks()

        self.threadProcess =  Thread(target = self.process)
        #self.calibrateMgSensor() #calibrate the Mg sensor 

        self.NiAlReader.start() #start reading
        self.threadProcess.start() #turn on voltage source and wait kill command
        #if self.MgOffset == None:
        
        _ = self.askUser()
        
        return 1

    def autoTune(self):
        """
        Manual tune source and ref. resistance with linear sequence
        """
        self.print_str.emit(self.list_properties())

        self.totalTime = 60
        self.kalman = KalmanF(self.BFeedback,offset=0.012, freq= self.FreqRet)
        self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)
        
        self.generateSlope()

        self.initTasks()

        self.threadProcess =  Thread(target = self.processAutoTune)
        #self.calibrateMgSensor() #calibrate the Mg sensor

        self.NiAlReader.start() #start reading 
        self.threadProcess.start() #turn on voltage source and wait kill command

        _ = self.askUser()

        return 1

    @pyqtSlot(float)
    def receiveScaler(self, data):
        with self.modelScaler.mutex:
            self.modelScaler.queue.clear()

        scaler = 1/self.model_coef*data
        self.modelScaler.put(scaler)

    def generateStepWave(self):
        """
        Generate step sequence
        """
        self.sequence = np.zeros(self.NSamples)

        start = int(self.FreqRet*5)
        end = int(self.totalTime*0.75*self.FreqRet)
        self.sequence[start:end] = 0.5

    def generateSlope(self):
        """
        Generate linear slope for calibration
        """
        #print("Generating slope")
        self.sequence = np.zeros(self.NSamples)
        self.cutOFF = 5*self.FreqRet

        offset = 0.1

        self.sequence[:self.cutOFF] = offset
        self.sequence[self.cutOFF:] = offset + np.arange(self.NSamples-self.cutOFF)*0.0001
        #self.sequence[-1] = 0

    def SinWave(self):
        """
        Generate linear slope for calibration
        """
        self.sequence = np.zeros(self.NSamples)

        offset = 0.25
        grad = 0.5
        freq = 0.05

        self.sequence[:1000] = offset
        self.sequence[1000:4000] = offset + grad*np.sin(2*np.pi*freq*np.arange(3000)*(1/100))
        #self.sequence[-1] = 0


"""
    
    def calibrateMgSensor(self):

        Find Mg sensor baseline
        Nsamples = 250
        
        #Create and close task
        calibR = nidaqmx.Task()
        calibR.ai_channels.add_ai_voltage_chan("Dev1/ai1", terminal_config = nidaqmx.constants.TerminalConfiguration.RSE, min_val = 0, max_val=5)  # has to match with chans_in
        datastream = AnalogSingleChannelReader(calibR.in_stream)
        calibR.start()

        dataBuffer = np.zeros(Nsamples)
        self.print_str.emit("Calibrating Mg Sensor")
        for i in range(Nsamples):
            dataBuffer[i] = datastream.read_one_sample()
            time.sleep(0.0001)
        
        calibR.stop()

        self.MgOffset = np.abs(np.mean(dataBuffer))

        self.print_str.emit("done sampling, Offset: {}".format(self.MgOffset))
        
        dataBuffer = np.empty(1)
        calibR.close()
        calibR = None

        return 1
"""