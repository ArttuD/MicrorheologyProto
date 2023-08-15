import pandas as pd
import numpy as np
import matplotlib.pyplot as plt 


class fit():

    def __init__(self, calibFlag=False):

        self.calibFlag = calibFlag

        self.path_driver = "./test/driver.npy"
        self.path_frame = "./test/FrameInfo_4800.npy"
        self.path_track = "./test/trackingData.pickle"

        file_driver = np.load(path_driver)
        file_frame = np.load(path_frame)

        with open(path_track, "rb") as input_file:
            file_track = cPickle.load(input_file)

        if self.calibFlag:
            file_calib = np.load(path_calib)

        self.s_ratio = int(100/40)

        self.creep_end = 
        self.creep_start = 

    def process(self):
        self.df_current = pd.DataFrame(self.file_driver.T, columns= ["time", "target", "current", "B", "offset"])
        self.df_current["time"] *= 1/100


        self.df_track = pd.DataFrame.from_dict(self.file_track)
        self.df_track["dx"] = scipy.ndimage.gaussian_filter(self.df_track["x"],5)
        self.df_track["dy"] = scipy.ndimage.gaussian_filter(self.df_track["y"],5)
        self.df_track["distance"] =self.df_track["dx"] + self.df_track["dy"]
        self.df_track["distance"] = self.df_track["distance"].values.max()- self.df_track["distance"] 
        self.df_track["time"] = self.df_track["t"]*1/40

    def fit(self):

    def maxwell_rel(self, t, E, tau):
        return E*np.exp(-t/tau)

    def kelvin_voigt(self, t, D, tau):
        return D*(1-np.exp(-t/tau))


    
