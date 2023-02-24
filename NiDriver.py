import nidaqmx
from nidaqmx.stream_readers import AnalogMultiChannelReader

from datetime import datetime
import time

import numpy as np

import threading
from queue import Queue

from tools.tools import *

from PyQt6.QtCore import pyqtSignal

class niDevice():
    data = pyqtSignal(np.array())

    def __init__(self, args, w):
        self.widowDown = False
        self.totalTime = args.time
        self.root = args.path
        self.samplingFreq = 100
        self.buffer_size = round(args.buffer_size_cfg*1)
        self.chans_in = args.chans_in
        self.que = Queue(maxsize=self.buffer_size)
        self.fileOut = "NiLog.csv"

        self.NiAlReader = nidaqmx.Task()
        self.NiAlWriter = nidaqmx.Task()
        self.NiDWriter = nidaqmx.Task()
        
        self.bufferIn = np.zeros((self.chans_in, self.buffer_size))
        self.values = np.zeros((self.chans_in,1))

        self.running = True
        self.startTime = None
        self.readerStreamIn = None

        self.kalman = KalmanF()

        self.writerFlag = False
        self.readerFlag = False

        self.iteration = -1

        self.generateStepWave()

        self.window = w


    def generateStepWave(self):
        self.sequence = np.zeros(self.samplingFreq*self.totalTime)
        self.sequence[10*self.samplingFreq:20*self.samplingFreq] = 1 

    def askUser(self):
        self.NiDWriter.write(True)

        input("Press Enter to stop the process")


        print("closing")
        self.running = False
        self.NiDWriter.write(False)

        while((self.writerFlag==False) | (self.readerFlag==False)):
            time.sleep(2)
            
            self.closeAll()

    def closeAll(self):
        self.NiAlReader.close()
        self.NiAlWriter.close()
        self.NiDWriter.close()
        self.writerFlag = False
        self.readerFlag = False
        self.widowDown = True
        exit(0)

    def cfg_AO_writer_task(self):
        self.NiAlWriter.ao_channels.add_ao_voltage_chan("Dev2/ao0", min_val=- 10.0, max_val=10.0)

    def cfg_DO_writer_task(self):
        self.NiDWriter.do_channels.add_do_chan("Dev2/port0/line0")

    def cfg_AL_reader_task(self):
        self.NiAlReader.ai_channels.add_ai_voltage_chan("Dev2/ai0", terminal_config = nidaqmx.constants.TerminalConfiguration.DIFF)
        self.NiAlReader.ai_channels.add_ai_voltage_chan("Dev2/ai1", terminal_config = nidaqmx.constants.TerminalConfiguration.RSE)  # has to match with chans_in
        self.NiAlReader.timing.cfg_samp_clk_timing(rate=self.samplingFreq, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                                            samps_per_chan=self.buffer_size)
        self.readerStreamIn = AnalogMultiChannelReader(self.NiAlReader.in_stream)
        self.NiAlReader.register_every_n_samples_acquired_into_buffer_event(self.buffer_size, self.readingCallback)

    def readingCallback(self, task_idx, event_type, num_samples, callback_data):
        if self.running:
            self.readerStreamIn.read_many_sample(self.bufferIn, self.samplingFreq, timeout = nidaqmx.constants.WAIT_INFINITELY)
            self.values = np.append(self.values,np.stack(((np.mean(self.bufferIn[0,:])*5/1023-2.6)/0.015,np.mean(self.bufferIn[1])), axis = 0).reshape(2,1), axis = 1)
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
                data  = self.kalman.filtering(measured, self.sequence[self.iteration])
                if data < -10:
                    data = -10
                elif data > 10:
                    data = 10
                self.NiAlWriter.write(data)
                setdata = np.array([self.iteration, self.sequence[self.iteration], measured[0], measured[1]])
                self.window.receiveData(setData)
            else:
                if self.running:
                    pass
                else:
                    self.writerFlag = True
                    break

    def start(self):
        print("starting the system")

        self.cfg_AL_reader_task()
        self.cfg_AO_writer_task()
        self.cfg_DO_writer_task()

        threadUser = threading.Thread(target = self.askUser)
        threadProcess = threading.Thread(target = self.process)

        #input("press enter to start")

        print("Generated tasks")
        self.start = datetime.now()

        threadUser.start()
        threadProcess.start()
        self.NiAlReader.start()
        self.NiAlWriter.start()
        self.NiDWriter.start()

        return 1



