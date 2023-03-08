import nidaqmx
from nidaqmx.stream_readers import AnalogMultiChannelReader

from datetime import datetime
import time

import numpy as np

import threading
from queue import Queue

from tools.tools import *
#from PyQt6.QtWidgets import PyQt_PyObject

from PyQt6.QtCore import pyqtSignal,QThread

class niDevice(QThread):
    setData = pyqtSignal(object)  

    def __init__(self, args):
        super().__init__()
        self.widowDown = False
        self.totalTime = args.time
        self.root = args.path
        self.samplingFreq = 100
        self.resistance1 = args.FirstResis 
        self.buffer_size = round(args.buffer_size_cfg*1)
        self.chans_in = args.chans_in
        self.que = Queue(maxsize=self.buffer_size)
        self.fileOut = "NiLog.csv"
        
        self.bufferIn = np.zeros((self.chans_in, self.buffer_size))
        self.values = np.zeros((self.chans_in,1))

        self.running = True
        self.startTime = None
        self.readerStreamIn = None

        self.kalman = KalmanF(offset=0.012)

        self.writerFlag = False
        self.readerFlag = False

        self.iteration = -1

        self.generateStepWave()

        self.currentToWrite = 0


    def generateStepWave(self):
        self.sequence = np.zeros(self.samplingFreq*self.totalTime)
        self.sequence[5:1000] = 1 

    def askUser(self):
        self.NiDWriter.write(True)

        while self.running:
            time.sleep(1)


        print("closing, current driver")
        self.running = False
        self.NiDWriter.write(False)

        while((self.writerFlag==False) & (self.readerFlag==False)):
            time.sleep(1)
            print("Waiting tasks to close")
        
        self.closeAll()

    def closeAll(self):
        self.NiAlReader.close()
        self.NiAlWriter.close()
        self.NiDWriter.close()
        self.writerFlag = False
        self.readerFlag = False
        self.widowDown = True
        

    def cfg_AO_writer_task(self):
        self.NiAlWriter.ao_channels.add_ao_voltage_chan("Dev1/ao0", min_val=- 10.0, max_val=10.0)

    def cfg_DO_writer_task(self):
        self.NiDWriter.do_channels.add_do_chan("Dev1/port0/line0")

    def cfg_AL_reader_task(self):
        self.NiAlReader.ai_channels.add_ai_voltage_chan("Dev1/ai0", terminal_config = nidaqmx.constants.TerminalConfiguration.DIFF)
        self.NiAlReader.ai_channels.add_ai_voltage_chan("Dev1/ai1", terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)  # has to match with chans_in
        self.NiAlReader.timing.cfg_samp_clk_timing(rate=self.samplingFreq, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                                            samps_per_chan=self.buffer_size)
        self.readerStreamIn = AnalogMultiChannelReader(self.NiAlReader.in_stream)
        self.NiAlReader.register_every_n_samples_acquired_into_buffer_event(self.buffer_size, self.readingCallback)

    def readingCallback(self, task_idx, event_type, num_samples, callback_data):
        if self.running:
            self.readerStreamIn.read_many_sample(self.bufferIn, self.samplingFreq, timeout = nidaqmx.constants.WAIT_INFINITELY)
            #self.values = np.append(self.values,np.stack(((np.mean(self.bufferIn[0,:])*5/1023-2.6)/0.015,np.mean(self.bufferIn[1])), axis = 0).reshape(2,1), axis = 1)
            self.values = np.append(self.values,np.stack((np.mean(self.bufferIn[0,:]),np.mean(self.bufferIn[1])), axis = 0).reshape(2,1), axis = 1)
            self.iteration += 1
            #print("putting que: ", self.values[:,-1])
            self.que.put(self.values[:,-1])
        else:
            print("shutting down")
            self.readerFlag = True
    
        return 0
    
    def process(self):
        while True:
            if self.que.empty() == False:
                measured = self.que.get()
                measured[0] = measured[0]/self.resistance1
                data  = self.kalman.filtering(measured, self.sequence[self.iteration])
                if data < -10:
                    data = -10
                elif data > 10:
                    data = 10
                
                self.NiAlWriter.write(data)
                #self.NiAlWriter.write(self.sequence[self.iteration])
                self.s  = np.array([self.iteration, self.sequence[self.iteration], measured[0], measured[1]])
                #print(self.s)
                self.setData.emit(self.s)
            else:
                if self.running:
                    pass
                else:
                    self.writerFlag = True
                    break

    def processTune(self):
        while True:
            if self.que.empty() == False:
                measured = self.que.get()
                measured[0] = measured[0]/self.resistance1
                data =  self.kalman.filtering(measured, self.currentToWrite)
                if data < -10:
                    data = -10
                elif data > 10:
                    data = 10
                self.NiAlWriter.write(data)
                #self.NiAlWriter.write(self.sequence[self.iteration])
                self.s  = np.array([self.iteration, data, measured[0], measured[1]])
                #print(self.s)
                self.setData.emit(self.s)
            else:
                if self.running:
                    pass
                else:
                    self.writerFlag = True
                    break

    def initTasks(self):

        self.NiAlReader = nidaqmx.Task()
        self.NiAlWriter = nidaqmx.Task()
        self.NiDWriter = nidaqmx.Task()
        print("starting the system")

        self.cfg_AO_writer_task()
        self.cfg_DO_writer_task()
        self.cfg_AL_reader_task()

    def tune(self):
        self.initTasks()

        threadUser = threading.Thread(target = self.askUser)
        threadProcess = threading.Thread(target = self.processTune)

        self.start = datetime.now()

        threadUser.start()
        threadProcess.start()
        self.NiAlReader.start()
        self.NiAlWriter.write(0)
        self.NiAlWriter.write(0)
        
        self.NiAlWriter.start()
        self.NiDWriter.start()

        return 1


    def start(self):

        self.initTasks()

        threadUser = threading.Thread(target = self.askUser)
        threadProcess = threading.Thread(target = self.process)

        self.start = datetime.now()

        threadUser.start()
        threadProcess.start()
        self.NiAlReader.start()
        self.NiAlWriter.write(0)
        self.NiAlWriter.write(0)
        
        self.NiAlWriter.start()
        self.NiDWriter.start()

        return 1



