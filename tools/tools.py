
from filterpy.kalman import KalmanFilter
import numpy as np

class KalmanF():

    def __init__(self):
        self.filter = KalmanFilter(dim_x=2, dim_z = 2)
        
        self.filter.x = np.array([0. , 0.])
        
        self.filter.B = np.array([[1,0],[0,1]])
        self.filter.H = np.array([1,1])
        self.filter.F = np.array([[1, 0],[0, 1]])
        
        self.filter.P = np.eye(N = self.filter.x.shape[0])
        self.filter.Q = np.eye(N = self.filter.x.shape[0])
        self.filter.R = np.array([[1e-2, 0],[0, 1]])

        self.PID = PIDcontroller()

    def filtering(self,z,aim):
        self.filter.predict()
        self.filter.update(z)
        out = self.PID.PID((self.filter.P[0,1]*z[0] + self.filter.P[0,1]*z[1])/(self.filter.P[0,1] + self.filter.P[0,1]), aim)
        return out

class PIDcontroller():
    
    def __init__(self):
        self.kp = 5
        self.ki = 1e-2
        self.kd = 1e-1
        self.integral = 0
        self.past=0
        self.error = None

    
    def PID(self, value, aim):
        self.error = aim-value

        prop = self.kp*self.error
        self.integral += self.ki*self.error
        derivative = self.kd*(self.past-self.error)
        self.past = self.error

        return prop+self.integral+derivative








