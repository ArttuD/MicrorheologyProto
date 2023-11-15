
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
        self.end = int(end_stamp/(1/fps))

    

    def download_numpy(self, path):

        df = pd.DataFrame(np.load(path).T, columns = ['index','aim','measured','Mg', 'scaler'])

        return df
    
    def download_pickle(self, path):

        with open(path, "rb") as input_file:
            file_track = cPickle.load(input_file)
            
        df_track = pd.DataFrame.from_dict(file_track)

        df_track["time"] = df_track["t"]*1/40

        df_track["x"] = scipy.ndimage.gaussian_filter(df_track["x"].max()-df_track["x"],5)
        df_track["y"] = scipy.ndimage.gaussian_filter(df_track["y"],5)
        df_track["x"] = df_track["x"].max() - df_track["x"]

        df_track["x"] -= df_track["x"].values[0]
        df_track["y"] -= df_track["y"].values[0]

        df_track["r"] = np.sqrt(df_track["x"]**2 + df_track["y"]**2)

        df_track["label"] = "0"
        df_track["type"] = "mg"

        df_track["x_ref"] = 0
        df_track["y_ref"] = 0 
        df_track["r_ref"] = 0
        df_track["r"] = 0
        df_track["tag_ref"] = "0"
        df_track["tag_mg"] = "0"

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

    def process_single(self, path_pickle, path_numpy, flag, count):

        if flag:
            df = self.download_pickle(path_pickle)
        else:
            df = self.download_json(path_pickle)
            
        df_driver = self.download_numpy(path_numpy)

        counter = 0
        for i in df.groupby(['tag_mg', "timestamps"]).mean().reset_index().groupby(["tag_mg"]):

            tags = i[0]
            data = i[1].reset_index()

            if counter == 0:
                fig, ax = plt.subplots(1,2, figsize=(10,7), facecolor="white")

                ax[1].set_title("Displacmeent summary")
                ax[1].plot(data["time"],data["x"], color = "red", label = "x-displacement")
                ax[1].plot(data["time"],data["y"], color = "black",label = "y-displacement")
                ax[1].plot(data["time"],data["r"], color = "purple", label = "r-displacement")
                self.start_time = data["time"].values[self.start]
                self.t_c = data["time"].values[self.end]
                ax[1].vlines(self.start_time, data["x"].min(), data["x"].max(), color = "blue", label = "Start creep", linestyle = "dashed")
                ax[1].vlines(self.t_c, data["x"].min(), data["x"].max(), color = "blue", label = "Start recovery", linestyle = "dashed")
                

                data["r"] = self.normalize(data["r"])
                data["time"] = self.normalize(data["time"])

                self.epsilon = np.mean(data["r"].values[self.end-5:self.end])
                self.start_time = data["time"].values[self.start]
                self.t_c = data["time"].values[self.end]

                ax[0]. set_title("Current and Magnetic Field flux")
                ax[0].set_xlabel("time (s)")
                ax[0].set_ylabel("i or B (A or T)")
                ax[0].plot(df_driver["index"],df_driver["measured"], label = "Measured i")
                ax[0].plot(df_driver["index"],df_driver["Mg"], label = "Measured B")

                ax[0].legend()

                plt.savefig(os.path.join(os.path.split(path_numpy)[0],"basics_{}.png".format(count)))
                plt.close()

            fig, ax = plt.subplots(2,2, figsize=(10,10), facecolor="white")
            plt.suptitle("Fit_{}".format(tags))

            t_hat_creep = data["time"][self.start:self.end]-data["time"].values[self.start]
            t_hat_rec = data["time"][self.end:]-data["time"].values[self.start]

            t_pred_creep = np.arange(t_hat_creep.min(),t_hat_creep.max(), 0.001)
            t_pred_rec = np.arange(t_hat_rec.min(),t_hat_rec.max(), 0.001)

            ax[0,0].set_title("Weibull")
            try:
                p, p_ = curve_fit(self.Weibull_creep, t_hat_creep, data["r"][self.start:self.end], bounds=([0,0,0,0], [np.inf,np.inf, 1, np.inf]), maxfev = 10000, method = "trf")
                #creep_label = "Creep: tau: {}".format(np.round(p[-1],5))
                #ax[0,0].plot(t_pred_creep, self.Weibull_creep(t_pred_creep,*p), label = creep_label, color = "red")
                p_1, p__ = curve_fit(self.Weibull_rec, t_hat_rec, data["r"][self.end:], bounds=([0,0,0,0], [np.inf,np.inf, 1, np.inf]), maxfev = 10000, method = "trf")
                creep_label = "Creep: tau: {}\nRel: tau: {}".format(np.round(p[-1],5),np.round(p[-1],5))
                ax[0,0].plot(np.append(t_pred_creep,t_pred_rec), np.append(self.Weibull_creep(t_pred_creep,*p),self.Weibull_rec(t_pred_rec,*p_1)), label = creep_label, color = "red")
            except:
                print("Weibull failed")
            #print("moi",data["time"][self.start:]-data["time"][self.start])
            #print("moi moi", data["r"][self.start:])
            ax[0,0].scatter(data["time"][self.start:]-data["time"][self.start], data["r"][self.start:], color = "black", label = "data")

            ax[0,0].set_xlabel("time (s)")
            ax[0,0].set_ylabel(r" $\epsilon$ ($\mu$m)")
            ax[0,0].legend(loc = "upper left",fontsize = 6)
            try:
                ax[0,1].set_title("Maxwell model")
                p, pcov = curve_fit(self.combine_maxwell, data["time"][self.start:] - data["time"][self.start], data["r"][self.start:], bounds=([0,0], [np.inf,np.inf]), maxfev = 10000, method = "trf")
                maxwell_label = "maxwell: \ntau: {}".format(np.round(p[-1]/p[1],5))
                ax[0,1].plot(np.append(t_pred_creep,t_pred_rec), np.append(self.maxwell_creep(t_pred_creep,*p), self.maxwell_rec(t_pred_rec,*p) ), label = maxwell_label,color = "red")
            except:
                print("maxwell failed")
            ax[0,1].scatter(data["time"][self.start:]-data["time"][self.start], data["r"][self.start:], color = "black", label = "data")
            ax[0,1].legend(loc = "upper left",fontsize = 6)
            ax[0,1].set_xlabel("time (s)")
            ax[0,1].set_ylabel(r" $\epsilon$ ($\mu$m)")

            ax[1,0].set_title("Burger model")
            try:
                p, pcov = curve_fit(self.combine_burger, data["time"][self.start:] - data["time"][self.start], data["r"][self.start:], bounds=([0,0,0,0], [np.inf,np.inf,np.inf,np.inf]), maxfev = 10000, method = "trf")
                burger_label = "Burger: \ntau: {}".format(np.round(p[-1]/p[1],5))
                ax[1,0].plot(np.append(t_pred_creep,t_pred_rec), np.append(self.burger_creep(t_pred_creep,*p), self.burger_rec(t_pred_rec,*p) ), label = burger_label,color = "red")
            except:
                print("Burger failed")
            ax[1,0].scatter(data["time"][self.start:]-data["time"][self.start], data["r"][self.start:], color = "black", label = "data")
            ax[1,0].legend(loc = "upper left",fontsize = 6)
            ax[1,0].set_xlabel("time (s)")
            ax[1,0].set_ylabel(r" $\epsilon$ ($\mu$m)")

            ax[1,1].set_title("Kelvin-Voigt")
            try: 
                p, pcov = curve_fit(self.combine_kelvin, data["time"][self.start:] - data["time"][self.start], data["r"][self.start:],bounds=([0,0], [np.inf,np.inf]), maxfev = 10000, method = "trf")
                burger_label = "kelvin: \ntau: {}".format(np.round(p[-1]/p[1],5))
                ax[1,1].plot(np.append(t_pred_creep,t_pred_rec), np.append(self.kelvin_creep(t_pred_creep,*p), self.kelvin_rec(t_pred_rec,*p) ), label = burger_label,color = "red")
            except:
                print("Kelvin failed")   
            ax[1,1].scatter(data["time"][self.start:]-data["time"][self.start], data["r"][self.start:], color = "black", label = "data")
            ax[1,1].legend(loc = "upper left",fontsize = 6)
            ax[1,1].set_xlabel("time (s)")
            ax[1,1].set_ylabel(r" $\epsilon$ ($\mu$m)")
            
            plt.savefig(os.path.join(os.path.split(path_numpy)[0],"fits_{}.png".format(tags)))
            plt.close()

            count += 1

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
    
    def combine_maxwell(self, x_tot, E_1, n_1):

        # single data reference passed in, extract separate data
        extract1 = x_tot[:(self.end-self.start)] # first data
        extract2 = x_tot[(self.end-self.start):] # second data

        result1 = self.maxwell_creep(extract1, E_1, n_1)
        result2 = self.maxwell_rec(extract2, E_1, n_1)

        return np.append(result1, result2)

    def normalize(self, data):
        return (data - np.min(data)) / (np.max(data) - np.min(data))
    
    def maxwell_rec(self, x, E_1, n_1 ):
        return  self.epsilon-1/E_1+0*x*n_1
    
    def maxwell_creep(self, x, E_1, n_1):
        return 1/E_1 + 1/n_1*x

    def burger_creep(self, x, E_1, E_2, n_1, n_2):
        return 1/E_1 + x/n_1 + 1/E_2*(1-np.exp(-x*E_1/n_2))
        
    def burger_rec(self, x, E_1, E_2, n_1, n_2):
        return self.t_c/n_1*(1-np.exp(-(x-self.t_c)*(E_2/n_2))) + (self.epsilon-1/E_1)*np.exp(-(x-self.t_c)*(E_2/n_2))
    
    def kelvin_creep(self, t, E_1, n_1):
        return 1/E_1*(1-np.exp(-t*E_1/n_1))

    def kelvin_rec(self, t, E_1, n_1):
        return 1/E_1*np.exp(-E_1/n_1*t)*(np.exp(-(E_1/n_1)*self.t_c)-1)

    def Weibull_creep(self,x, e_i, e_c, beta, tau):
        return e_i + e_c*(1-np.exp(-(x/tau)**beta))

    def Weibull_rec(self,x, e_r, e_f, beta, tau):
        return e_f + e_r*(np.exp(-(x/tau)**beta))

def load_csv(root):
    
    df = pd.read_csv(root, sep = ",", skiprows= [0,1], names = ["x", "y", "By", "Bx"])
    #drop not needed
    df.drop(df[df["y"].values < (0.0035 - 1536*m)].index, inplace = True)
    df.drop(df[df["x"].values > 0.00054].index, inplace = True)

    B = np.sqrt(df["By"]**2 + df["Bx"]**2)
    
    rows = df["x"].unique()
    columns = df["y"].unique()

    return B, rows, columns


def findPoint(x,y):

    x_idx = np.argmin(np.abs((rows-np.round(x,5))))
    y_idx = np.argmin(np.abs((columns-np.round(y,5))))
    current = B[x_idx, y_idx]
    error = prev_ - current
    prev_ = current
    

def fit_calibration(df):
    cut_off = 5*100
    maxIndex = np.where(df["measured"].values >=0.5)[0][0]
    x = df["measured"].values[cut_off:maxIndex]
    y = df["Mg"].values[cut_off:maxIndex]
    k, b  = np.polyfit(x,y, 1)
    
    return x, y, k, b, cut_off, maxIndex

def fetchDataNPY(paths):
    for i in range(len(paths)):
        datas = np.load(paths[i])
        print(datas, "\ni",i )
        if i == 0:
            df = pd.DataFrame(datas.T, columns = ['index','aim','measured','Mg'])
            df["measurement"] = i
        else:
            dftemp = pd.DataFrame(datas.T, columns = ['index','aim','measured','Mg'])
            dftemp["measurement"] = i
            df = pd.concat([df, dftemp])
    
    return df

def fetchDataCSV(paths):
    
    for i in range(len(paths)):
        if i == 0:
            df = pd.read_csv(paths[i], index_col = False)
            df["measurement"] = os.path.split(paths[i])[1].split("_")[1][:-4]
        else:
            dftemp =  pd.read_csv(paths[i], index_col = False)
            dftemp["measurement"] = os.path.split(paths[i])[1].split("_")[1][:-4]
            df = pd.concat([df, dftemp])
    return df

def func(x, a, b):
    return a*np.tanh(x*b)

def kelvin(t,tau,D):
    return D*(1-np.exp(-t/tau))

def maxwell(t, D, tau):
    return D*(1+t/tau)

def maxwell_rel(t, E, tau ):
    return E*np.exp(-t/tau) 

def burger_model(x, E_1, E_2, n_1, n_2):
    return 1/E_1 + x/n_1 + 1/E_2*(1-np.exp(-x*E_1/n_2))

def burger_rel(x, E_1, E_2, n_1, n_2):
    t_c = 1
    return t_c/n_1*(1- np.exp(-(x-t_c)/(n_2/E_2))) + (1/E_1)*np.exp(-(x-t_c)/(n_2/E_2))

def Weibull_creep(x, e_i, e_c, beta, tau):
    return e_i + e_c*(1-np.exp(-(x/tau)**beta))

def Weibull_rel(x, e_r, e_f, beta, tau):
    return e_f + e_r*(np.exp(-(x/tau)**beta))


def download_pickle(path):

    with open(path, "rb") as input_file:
        file_track = cPickle.load(input_file)
        
    df_track = pd.DataFrame.from_dict(file_track)

    df_track["time"] = df_track["t"]*1/40

    df_track["x"] = scipy.ndimage.gaussian_filter(df_track["x"].max()-df_track["x"],5)
    df_track["y"] = scipy.ndimage.gaussian_filter(df_track["y"],5)
    df_track["x"] = df_track["x"].max() - df_track["x"]

    df_track["x"] -= df_track["x"].values[0]
    df_track["y"] -= df_track["y"].values[0]

    df_track["r"] = np.sqrt(df_track["x"]**2 + df_track["y"]**2)

    df_track["label"] = "0"
    df_track["type"] = "mg"

    df_track["x_ref"] = 0
    df_track["y_ref"] = 0 
    df_track["r_ref"] = 0
    df_track["r"] = 0
    df_track["tag_ref"] = "0"
    df_track["tag_mg"] = "0"

    return df_track

def download_json(path):

    start_stamp = 5
    end_stamp = 90
    df_array = []
    fps = 40
    start = int(start_stamp/(1/fps))
    end = int(end_stamp/(1/fps))

    with open(path, "rb") as input_file:
        file_dict = json.load(input_file)

    for i in file_dict.keys():
                
        parts = i.split("_")
        if parts[0] == "ref":
            if parts[1] == "0":
                temp_df = pd.DataFrame.from_dict(file_dict[i])

                temp_df["time"] = temp_df["timestamps"]*1/fps

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

                temp_df["time"] = temp_df["timestamps"]*1/fps
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

                temp_df["time"] = temp_df["timestamps"]*1/fps
                temp_df["x"] -= temp_df["x"].values[0]
                temp_df["y"] -= temp_df["y"].values[0]
                df_mg = temp_df
                df_mg["label"] = parts[1]
            else:
                temp_df = pd.DataFrame.from_dict(file_dict[i])
                temp_df["label"] = parts[1]

                temp_df["time"] = temp_df["timestamps"]*1/fps
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