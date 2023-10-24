
from filterpy.kalman import KalmanFilter
import numpy as np
import pandas as pd

class KalmanF():

    def __init__(self, feedBackMode, offset, freq):
        self.filter = KalmanFilter(dim_x=2, dim_z = 2)
        
        self.filter.x = np.array([0. , 0.])
        
        self.filter.B = np.array([[1,0],[0,1]])
        self.filter.H = np.array([1,1])
        self.filter.F = np.array([[1, 0],[0, 1]])
        
        self.filter.P = np.eye(N = self.filter.x.shape[0])
        self.filter.Q = np.eye(N = self.filter.x.shape[0])
        self.filter.R = np.array([[1e-2, 0],[0, 1000]])
        
        if feedBackMode:
            self.PID = PIDMGcontroller(freq)
        else:
            self.PID = PIDcontroller(freq)

    def filtering(self,z,aim):
        self.filter.predict()
        self.filter.update(z)
        #out = self.PID.PID((self.filter.P[0,1]*z[0] + self.filter.P[1,1]*z[1])/(self.filter.P[0,1] + self.filter.P[1,1]), aim)
        #print("going to PID",z[0], aim)
        out = self.PID.PID(z, aim)

        return out

class PIDcontroller():
    
    def __init__(self, freq):
        #print("Created Current controller")
        self.kp = 20#25
        self.ki = 290 #30
        self.kd = 0.00002 #0.075
        self.integral = 0
        self.past=0
        self.error = None
        self.emaFilter = EMA(0.50)

        self.dt = 1/freq
    
    def PID(self, meas, aim):
        value = self.emaFilter.filterNow(meas[0])
        self.error = aim-value

        prop = self.kp*self.error
        self.integral += self.ki*self.error*self.dt
        derivative = self.kd*(self.past-self.error)/self.dt
        self.past = self.error

        return prop +self.integral+derivative
    
def remap(value, OldMin, OldMax, NewMin, NewMax):
    OldRange = (OldMax - OldMin)  
    NewRange = (NewMax - NewMin)  
    NewValue = (((value - OldMin) * NewRange) / OldRange) + NewMin
    return NewValue


class PIDMGcontroller():
    
    def __init__(self,freq):
        print("Created MG controller")
        self.kp = 21
        self.ki = 600
        self.kd = 0.0000007

        self.integral = 0
        self.past=0
        self.error = None
        self.emaFilter = EMA(0.40)
        self.dt = 1/freq
    
    
    def PID(self, meas, aim):
        value = self.emaFilter.filterNow(meas[1])
        self.error = aim-value

        prop = self.kp*self.error
        self.integral += self.ki*self.error*self.dt
        derivative = self.kd*(self.past-self.error)/self.dt
        self.past = self.error

        return prop+self.integral+derivative

        return prop+self.integral+derivative+aim



class EMA():

    def __init__(self, alpha):
        self.alpha = alpha
        self.prev = 0

    def filterNow(self,sample):
        out = self.alpha*sample + (1-self.alpha)*self.prev
        self.prev = out
        return out


class alphaBeta():

    def __init__(self,alpha,beta,dt):
        self.a = alpha
        self.b = beta

        self.xk_1 = 0
        self.vk_1 = 0

        self.vk = 0

        self.dt = 1/dt

    def filterNow(self,xm):

        self.xk = self.xk_1 + (self.vk_1*self.dt);
        self.vk = self.vk_1

        self.rk = xm - self.xk

        self.xk += self.a*self.rk
        self.vk += (self.b*self.rk)/self.dt

        self.xk_1 = self.xk
        self.vk_1 = self.vk

        return self.xk_1

