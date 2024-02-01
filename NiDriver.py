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


    def __init__(self, args, ctr, value_ctr):
        super().__init__()
        self.args = args
        self.ctr = ctr
        self.value_ctr = value_ctr

        self.mode = None
        self.saveFlag = None
        self.feedBack = None
        self.resistance1 = self.value_ctr["res"]

        self.writeQue = Queue(maxsize=100)
        self.resOneQue = Queue(maxsize=100)

        #parse arguments
        self.totalTime = args.time
        self.root = args.path
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

        self.T2i_coef = 0.28

        self.scaler =  0
        self.sequence = None
        self.iteration = 0

        self.properties = None

    def list_properties(self):
        self.properties = "Saving to {}. Running with Hall B slope: {}, offset {}, B2i conversion {}, Resistance {}, and model scaling {}".format( self.root, self.Mgcoef, self.MgOffset, self.T2i_coef, self.resistance1, self.T2i_coef)

    
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
        if (self.ctr["close"] == False):
            self.readerStreamIn.read_many_sample(self.bufferIn, self.buffer_size, timeout = nidaqmx.constants.WAIT_INFINITELY)
            self.values = np.append(self.values,np.stack((np.mean(self.bufferIn[0,:]),np.abs(np.mean(self.bufferIn[1,:]))), axis = 0).reshape(2,1), axis = 1)
            self.que.put(self.values[:,-1])
            self.iteration += 1
            if (self.iteration == self.NSamples):
                self.que.put(np.array((-1,-1,-1)))
                self.NiAlReader.stop()
        else:
            self.que.put(np.array((-1,-1,-1)))
            self.NiAlReader.stop()
                
            
        return 0

    def checkLimits(self, data):
        """
        Check that writing value is in the limits
        """
        if data < -10:
            data = -10
        elif data > 10:
            data = 10

        return data

    def write_empty(self):

        for i in range(5):
            self.NiAlWriter.write(0,1000)

    def run(self):

        self.mode = self.ctr["mode"]
        self.saveFlag = self.ctr["save"]
        self.feedBack = self.ctr["feedback"]
        self.B_feedback = self.ctr["Bcontrol"]
        self.resistance1 = self.value_ctr["res"]


        self.kalman = KalmanF(self.B_feedback,offset=0.012, freq= self.FreqRet)

        self.initTasks()
        """
        Turn on relay and wait for stop command
        """
        self.NiDWriter.start()
        self.NiDWriter.write(True, 1000)

        self.NiAlWriter.start()
        self.write_empty()
        
        #self.calibrateMgSensor() #calibrate the Mg sensor 

        self.NiAlReader.start() #start reading

        if self.mode == 1:
            #manual
            self.NSamples = np.inf #Continue until stopped
            ret = self.processTune()
        elif self.mode == 0:
            #auto
            self.totalTime = 60
            self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)
            self.generateSlope()
            self.processAutoTune()
        elif self.mode == 2:
            #meas
            self.totalTime = self.args.time
            self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)
            self.generateStepWave()
            #self.SinWave()
            ret = self.process()

        self.write_empty()
        self.NiDWriter.write(False, 1000)
        

        self.NiDWriter.stop() 
        self.NiAlWriter.stop()
        self.NiAlReader.close()
        self.NiAlWriter.close()
        self.NiDWriter.close()

        #if self.modelFlag:
        #    with self.modelScaler.mutex:
        #        self.modelScaler.queue.clear()

        self.iteration = 0 
        self.totalTime = self.args.time
        self.NSamples = int(self.samplingFreq*self.totalTime/self.buffer_size)
        self.value_ctr["scaler"] = 0

        #print("waiting to close driver")
        while self.ctr["close"] == False:
            time.sleep(0.75)
             
        self.ctr["close"] = False
        #print("Ni exit")

        return 1
    
    def fetchQue(self, measured, writeC):
        """
        Fetch from que and send to filter+PID
        """
        #print(measured, self.scaler)
        measured[0] = 2*measured[0]
        measured[1] = (measured[1]-self.MgOffset)/self.Mgcoef #print("feedback")
        if self.feedBack:
            #print(measured[0], measured[1], (writeC-writeC*self.scaler)*self.resistance1)
            kalmanOut =  self.kalman.filtering(np.array([measured[0], measured[1]/self.T2i_coef]), (writeC-writeC*self.scaler)*self.resistance1)
            data = kalmanOut #*self.resistance1
        else:
            data = (writeC-writeC*self.scaler)*self.resistance1
        
        #print("data", data, "\nscaler", self.scaler, "resistance", self.resistance1, "Data in:", writeC)
        return measured,data

    
    def process(self):
        """
        Pipe measurement
        """
        
        QueIndex = 0
        #print("scaler dict:", self.ctr["scaler"])
        
        while True:
            if (self.que.empty() == False) :
                measured = self.que.get()
                if measured[0] == -1:
                    #print("ni break")
                    break
                else:
                    #fetch from que
                    measured, data = self.fetchQue(measured, self.sequence[QueIndex])

                    #data =  self.sequence[QueIndex]
                    data = self.checkLimits(data)
                    
                    #Emit Qt
                    if QueIndex%10 == 0:
                        self.s  = np.array([QueIndex, self.sequence[QueIndex], measured[0]/self.resistance1, measured[1]])
                        self.setData.emit(self.s)
                        coef = self.value_ctr["scaler"]
                        self.scaler = coef#1/self.T2i_coef*coef

                    #if model_que.is_empty() == False:
                    #    self.scaler = self.model_que.get()/self.T2i_coef
                
                    #write
                    self.NiAlWriter.write(data,1000)

                    #Collect
                    #print(self.scaler)
                    self.NpyStorage[0,QueIndex] = QueIndex*1/self.FreqRet
                    self.NpyStorage[1,QueIndex] = self.sequence[QueIndex]
                    self.NpyStorage[2,QueIndex] = measured[0]/self.resistance1
                    self.NpyStorage[3,QueIndex] = measured[1]
                    self.NpyStorage[4,QueIndex] = self.scaler

                    QueIndex += 1
            else:
                time.sleep(0.001)

        self.write_empty()
        #Save log
        #print("Saving Flag", self.saveFlag)
        np.save(os.path.join(self.root,"driver_{}.npy".format(datetime.date.today())), self.NpyStorage)

        return 1
        
    def processTune(self):
        """
        Tuning measurement
        """

        QueIndex = 0

        self.currentToWrite = self.value_ctr["cur"]
        self.resistance1 = self.value_ctr["res"]

        while True:
            if (self.que.empty() == False) :
                
                measured = self.que.get()
                if measured[0] == -1:
                    break
                else:
                    #fetch from que
                    measured, data = self.fetchQue(measured, self.currentToWrite)         
                    data = self.checkLimits(data)
                    
                    #write
                    #print(data)
                    self.NiAlWriter.write(data,1000)
                    
                    #Emit to Qt
                    if QueIndex%10 == 0:
                        self.s  = np.array([self.iteration, self.currentToWrite, measured[0]/self.resistance1, measured[1]])
                        self.setData.emit(self.s)

                        self.currentToWrite = self.value_ctr["cur"]
                        self.resistance1 = self.value_ctr["res"]

                    QueIndex += 1
            else:
                time.sleep(0.001)

        self.write_empty()
        return 1

    def processAutoTune(self):
        """
        Pipe autocalibration
        """
        
        QueIndex = 0
        while True:
            if (self.que.empty() == False):

                measured = self.que.get()
                if measured[0] == -1:
                    break
                else:
                    #fetch from que
                    measured, data = self.fetchQue(measured,self.sequence[QueIndex])   
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
                time.sleep(0.001)

        self.write_empty()
        np.save(os.path.join(self.root,"calib_{}.npy".format(datetime.date.today())), self.NpyStorage)
        if QueIndex >= (self.NSamples-1):
            self.fitCalibration()

        return 1
            

    def fitCalibration(self):

        maxIndex = np.where(self.NpyStorage[2,:] >=0.5)[0][0]

        x = self.NpyStorage[2,self.cutOFF:maxIndex]*self.resistance1
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