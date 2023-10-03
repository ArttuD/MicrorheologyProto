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

        self.creep_end = None
        self.creep_start = int(10000/100*5/self.s_ratio)

    def process(self):
        self.df_current = pd.DataFrame(self.file_driver.T, columns= ["time", "target", "current", "B", "offset"])
        self.df_current["time"] *= 1/100


        self.df_track = pd.DataFrame.from_dict(self.file_track)
        self.df_track["dx"] = scipy.ndimage.gaussian_filter(self.df_track["x"],5)
        self.df_track["dy"] = scipy.ndimage.gaussian_filter(self.df_track["y"],5)
        self.df_track["distance"] =self.df_track["dx"] + self.df_track["dy"]
        self.df_track["distance"] = self.df_track["distance"].values.max()- self.df_track["distance"] 
        self.df_track["time"] = self.df_track["t"]*1/40

        self.creep_end = int(df_track["t"].max()*0.75)

    def fit(self):
        fig, ax = plt.sub
        plt.plot(df_track["time"], df_track["distance"], label = "data")
        t_hat = np.arange(df_track["time"][start_creep:end_creep].min(),df_track["time"][start_creep:end_creep].max(), 0.001)

        p, p_ = curve_fit(kelvin, df_track["time"][start_creep:end_creep]-df_track["time"][start_creep:end_creep].values[0], df_track["distance"][start_creep:end_creep], method = "dogbox")
        plt.plot(t_hat, kelvin(t_hat-t_hat[0],*p), label = "Kelvin-Voigt")

    def maxwell_rel(self, t, E, tau):
        return E*np.exp(-t/tau)

    def kelvin_voigt(self, t, D, tau):
        return D*(1-np.exp(-t/tau))


    
