
import numpy as np
import matplotlib.pyplot as plt
import os

import _pickle as cPickle
import json

import scipy
from scipy.optimize import curve_fit

import pandas as pd

class fit():

    def __init__(self, fps, start_stamp, end_stamp):

        self.epsilon = None
        self.t_c = None

        self.start_time = start_stamp
        self.end_time = end_stamp

        self.fps = fps

        self.start = int(start_stamp/(1/fps))
        self.end =int(end_stamp/(1/fps))

    def download_numpy(self, path):

        df = pd.DataFrame(np.load(path).T, columns = ['index','aim','measured','Mg', 'scaler'])

        return df
    
    def download_pickle(self, path):

        with open(path, "rb") as input_file:
            file_track = cPickle.load(input_file)
            
        df_track = pd.DataFrame.from_dict(file_track)

        df_track["time"] = df_track["t"]*1/40

        df_track["x"] = scipy.signal.detrend(scipy.ndimage.gaussian_filter(df_track["x"].max()-df_track["x"],5))
        df_track["y"] = scipy.signal.detrend(scipy.ndimage.gaussian_filter(df_track["y"],5))
        df_track["x"] -= df_track["x"].values[0]
        df_track["y"] -= df_track["y"].values[0]

        df_track["r"] = np.sqrt(df_track["x"]**2 + df_track["y"]**2)
        df_track["r"] -= df_track["r"].values[0]

        return df_track

    def download_json(self, path):

        df_array = []

        with open(path, "rb") as input_file:
            file_dict = json.load(input_file)

        for i in file_dict.keys():
                    
            parts = i.split("_")
            if parts[0] == "ref":
                if parts[1] == "0":
                    temp_df = pd.DataFrame.from_dict(file_dict[i])

                    temp_df["time"] = temp_df["timestamps"]*1/self.fps

                    temp_df["x"] = scipy.ndimage.gaussian_filter(temp_df["x"],5)
                    temp_df["y"] = scipy.ndimage.gaussian_filter(temp_df["y"],5)
                    temp_df["x"] = temp_df["x"].max() - temp_df["x"]


                    temp_df["x"] -= temp_df["x"].values[0]
                    temp_df["y"] -= temp_df["y"].values[0]
                    df_ref = temp_df
                    df_ref["label"] = parts[1]
                else:
                    temp_df = pd.DataFrame.from_dict(file_dict[i])
                    temp_df["label"] = parts[1]

                    temp_df["time"] = temp_df["timestamps"]*1/self.fps
                    temp_df["x"] = scipy.ndimage.gaussian_filter(temp_df["x"],5)
                    temp_df["y"] = scipy.ndimage.gaussian_filter(temp_df["y"],5)
                    temp_df["x"] = temp_df["x"].max() - temp_df["x"]

                    temp_df["x"] -= temp_df["x"].values[0]
                    temp_df["y"] -= temp_df["y"].values[0]

                    df_ref = pd.concat((df_ref,temp_df))
                df_ref["type"] = "ref"
                #df_ref["label"] = parts[1]
            else:
                if parts[1] == "0":
                    temp_df = pd.DataFrame.from_dict(file_dict[i])
                    temp_df["x"] = scipy.ndimage.gaussian_filter(temp_df["x"],5)
                    temp_df["y"] = scipy.ndimage.gaussian_filter(temp_df["y"],5)
                    temp_df["x"] = temp_df["x"].max() - temp_df["x"]

                    temp_df["time"] = temp_df["timestamps"]*1/self.fps
                    temp_df["x"] -= temp_df["x"].values[0]
                    temp_df["y"] -= temp_df["y"].values[0]
                    df_mg = temp_df
                    df_mg["label"] = parts[1]
                else:
                    temp_df = pd.DataFrame.from_dict(file_dict[i])
                    temp_df["label"] = parts[1]

                    temp_df["time"] = temp_df["timestamps"]*1/self.fps
                    temp_df["x"] = scipy.ndimage.gaussian_filter(temp_df["x"],5)
                    temp_df["y"] = scipy.ndimage.gaussian_filter(temp_df["y"],5)
                    temp_df["x"] = temp_df["x"].max() - temp_df["x"]

                    temp_df["x"] -= temp_df["x"].values[0]
                    temp_df["y"] -= temp_df["y"].values[0]
                    df_mg = pd.concat((df_mg,temp_df))
                
                df_mg["type"] = "mg"

        for i in df_ref.groupby((["label"])):
            tags = i[0][0]
            frame = i[1]
            
            for j in df_mg.groupby((["label"])):
                tags_mg = j[0][0]
                #print(tags_mg)
                frame_mg = j[1]
                frame_mg["x_ref"] = frame_mg["x"] -  frame["x"]
                frame_mg["y_ref"] = frame_mg["y"] -  frame["y"]

                frame_mg["x_ref"] -= frame_mg["x_ref"].values[self.start]
                frame_mg["y_ref"] -= frame_mg["y_ref"].values[self.start]
                
                frame_mg["r_ref"] = scipy.signal.detrend(np.sqrt(frame_mg["x_ref"]**2 +  frame_mg["y_ref"]**2))
                frame_mg["r_ref"] -= frame_mg["r_ref"].values[self.start]

                frame_mg["r"] = np.sqrt(frame_mg["x"]**2 +  frame_mg["y"]**2)
                frame_mg["r"] -= frame_mg["r"].values[self.start]

                frame_mg["tag_ref"] = tags
                frame_mg["tag_mg"] = tags_mg
                df_array.append(frame_mg)

        for count, k in enumerate(df_array):
            if count == 0:
                df_tot = k
            else:
                df_tot = pd.concat((df_tot,k))
        
        return df_mg, df_ref, df_tot

    def process_single(self, path_pickle, path_numpy):

        df = self.download_pickle(path_pickle)
        df_driver = self.download_numpy(path_numpy)

        self.epsilon = np.mean(df["r"].values[self.end-20:self.end])
        self.t_c = self.end_time

        t_hat_creep = df["time"][self.start:self.end]-df["time"].values[self.start]
        t_hat_rec = df["time"][self.end:]-df["time"].values[self.start]

        t_pred_creep = np.arange(t_hat_creep.min(),t_hat_creep.max(), 0.001)
        t_pred_rec = np.arange(t_hat_rec.min(),t_hat_rec.max(), 0.001)

        fig, ax = plt.subplots(1,2, figsize=(10,7), facecolor="white")

        ax[0]. set_title("Current and Magnetic Field flux")
        ax[0].set_xlabel("time (s)")
        ax[0].set_ylabel("i or B (A or T)")
        ax[0].plot(df_driver["index"],df_driver["measured"], label = "Measured i")
        ax[0].plot(df_driver["index"],df_driver["Mg"], label = "Measured B")

        ax[1].set_title("Displacmeent summary")
        ax[1].plot(df["time"],df["x"], color = "red", label = "x-displacement")
        ax[1].plot(df["time"],df["y"], color = "black",label = "y-displacement")
        ax[1].plot(df["time"],df["r"], color = "purple", label = "r-displacement")
        ax[1].vlines(self.start_time, df["x"].min(), df["x"].max(), color = "blue", label = "Start creep", linestyle = "dashed")
        ax[1].vlines(self.t_c, df["x"].min(), df["x"].max(), color = "blue", label = "Start recovery", linestyle = "dashed")
        
        plt.savefig(os.path.join(os.path.split(path_pickle)[0],"basics.png"))
        plt.close()

        fig, ax = plt.subplots(2,2, figsize=(10,10), facecolor="white")

        ax[0,0].set_title("Weibull")
        p, p_ = curve_fit(self.Weibull_creep, t_hat_creep, df["r"][self.start:self.end], maxfev = 10000)
        creep_label = "Creep: tau: {}".format(np.round(p[-1],5))
        ax[0,0].plot(t_pred_creep, self.Weibull_creep(t_pred_creep,*p), label = creep_label, color = "red")

        p, p_ = curve_fit(self.Weibull_rec, t_hat_rec, df["r"][self.end:], maxfev = 10000)
        creep_label = "Rel: tau: {}".format(np.round(p[-1],5))

        ax[0,0].plot(t_pred_rec, self.Weibull_rec(t_pred_rec,*p), label = creep_label, color = "orange")
        ax[0,0].scatter(df["time"][self.start:]-df["time"][self.start], df["r"][self.start:], color = "black", label = "data")
        ax[0,0].set_xlabel("time (s)")
        ax[0,0].set_ylabel(r" $\epsilon$ ($\mu$m)")
        ax[0,0].legend(loc = "upper left",fontsize = 6)

        ax[0,1].set_title("Maxwell")
        p, p_ = curve_fit(self.maxwell_creep, t_hat_creep, df["r"][self.start:self.end], maxfev = 10000)
        creep_label = "Creep: tau: {}".format(np.round(p[1]/p[0],5))
        ax[0,1].plot(t_pred_creep, self.maxwell_creep(t_pred_creep,*p), label = creep_label, color = "red")

        p, p_ = curve_fit(self.maxwell_rec, t_hat_rec, df["r"][self.end:], maxfev = 10000)
        creep_label = "Rel: tau: {}".format(np.round(p[1]/p[0],5))
        ax[0,1].plot(t_pred_rec, self.maxwell_rec(t_pred_rec,*p), label = creep_label, color = "orange")
        ax[0,1].scatter(df["time"][self.start:]-df["time"][self.start], df["r"][self.start:], color = "black", label = "data")
        ax[0,1].set_xlabel("time (s)")
        ax[0,1].set_ylabel(r" $\epsilon$ ($\mu$m)")
        ax[0,1].legend(loc = "upper left",fontsize = 6)

        ax[1,0].set_title("Burger model")
        p, pcov = curve_fit(self.combine_burger, df["time"][self.start:] - df["time"][self.start], df["r"][self.start:], maxfev = 100000)

        burger_label = "Burger: \ntau: {}".format(np.round(p[-1]/p[1],5))
        ax[1,0].plot(np.append(t_pred_creep,t_pred_rec), np.append(self.burger_creep(t_pred_creep,*p), self.burger_rec(t_pred_rec,*p) ), label = burger_label,color = "red")
        ax[1,0].scatter(df["time"][self.start:]-df["time"][self.start], df["r"][self.start:], color = "black", label = "data")
        ax[1,0].legend(loc = "upper left",fontsize = 6)
        ax[1,0].set_xlabel("time (s)")
        ax[1,0].set_ylabel(r" $\epsilon$ ($\mu$m)")

        ax[1,1].set_title("Kelvin-Voigt")
        p, pcov = curve_fit(self.combine_kelvin, df["time"][self.start:] - df["time"][self.start], df["r"][self.start:], maxfev = 100000)

        burger_label = "kelvin: \ntau: {}".format(np.round(p[-1]/p[1],5))
        ax[1,1].plot(np.append(t_pred_creep,t_pred_rec), np.append(self.kelvin_creep(t_pred_creep,*p), self.kelvin_rec(t_pred_rec,*p) ), label = burger_label,color = "red")
        ax[1,1].scatter(df["time"][self.start:]-df["time"][self.start], df["r"][self.start:], color = "black", label = "data")

        ax[1,1].legend(loc = "upper left",fontsize = 6)
        ax[1,1].set_xlabel("time (s)")
        ax[1,1].set_ylabel(r" $\epsilon$ ($\mu$m)")
        
        plt.savefig(os.path.join(os.path.split(path_pickle)[0],"fits.png"))
        plt.close()

    def combine_burger(self, x_tot, E_1, E_2, n_1, n_2):

        # single data reference passed in, extract separate data
        extract1 = x_tot[:(self.end-self.start)] # first data
        extract2 = x_tot[(self.end-self.start):] # second data

        result1 = self.burger_creep(extract1, E_1, E_2, n_1, n_2)
        result2 = self.burger_rec(extract2, E_1, E_2, n_1, n_2)
        return np.append(result1, result2)
    
    def combine_kelvin(self, x_tot, E_1, n_1):

        # single data reference passed in, extract separate data
        extract1 = x_tot[:(self.end-self.start)] # first data
        extract2 = x_tot[(self.end-self.start):] # second data

        result1 = self.kelvin_creep(extract1, E_1, n_1)
        result2 = self.kelvin_rec(extract2, E_1, n_1)

        return np.append(result1, result2)

    def maxwell_rec(self, x, E_1, n_1 ):
        return  self.epsilon*np.exp(-(x-self.t_c)*E_1/n_1)
    
    def maxwell_creep(self, x, E_1, n_1):
        return 1/E_1 + 1/n_1*x

    def burger_creep(self, x, E_1, E_2, n_1, n_2):
        return 1/E_1 + x/n_1 + 1/E_2*(1-np.exp(-x*E_1/n_2))
        
    def burger_rec(self, x, E_1, E_2, n_1, n_2):
        return self.t_c/n_1*(1-np.exp(-(x-self.t_c)*(E_2/n_2))) + (self.epsilon-1/E_1)*np.exp(-(x-self.t_c)*(E_2/n_2))
    
    def kelvin_creep(self, t, E_1, n_1):
        return 1/E_1*(1-np.exp(-t*E_1/n_1))

    def kelvin_rec(self, t, E_1, n_1):
        return self.epsilon*(np.exp(-(t-self.t_c)*E_1/n_1))

    def Weibull_creep(self,x, e_i, e_c, beta, tau):
        return e_i + e_c*(1-np.exp(-(x/tau)**beta))

    def Weibull_rec(self,x, e_r, e_f, beta, tau):
        return e_f + e_r*(np.exp(-(x/tau)**beta))


    
