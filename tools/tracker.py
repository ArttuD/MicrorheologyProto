import numpy as np
import cv2
import pickle
import os
import datetime

class tracker():

    def __init__(self,boundaries):
        self.frame = None
        self.refPt = []
        self.fBound = []
        #rescaled = 640x480 and init 2048x1536
        self.w = None
        self.h = None
        self.k = None

        self.x_scale = 2048/480
        self.y_scale = 1536/480

        # change from screen coordinates to image coordinates
        self.x = boundaries[0][0][0]*self.x_scale
        self.x2 = boundaries[1][0][0]*self.x_scale
        self.y = boundaries[0][0][1]*self.y_scale
        self.y2 = boundaries[1][0][1]*self.y_scale

        self.xOrg = boundaries[0][0][0]*self.x_scale
        self.yOrg = boundaries[0][0][1]*self.y_scale
        self.binary = None
 
        """
        self.x = int(boundaries[0][0][0])
        self.x2 = int(boundaries[0][1][0])
        self.y = int(boundaries[0][0][1])
        self.y2 = int(boundaries[0][1][1])
        """

        self.viz = True
        self.cap = None
        self.col = (0, 255, 0)
        self.Dict = {"x":[],"y":[],"t":[]}
        self.t = 0
    
    def _round(self,x):
        return int(np.floor(x))

    def coords_to_int(self):
        # round to closest integer
        return self._round(self.x),self._round(self.x2),\
                self._round(self.y),self._round(self.y2)

    def main(self, img):
        self.frame = np.copy(img)
        self.w,self.h = self.frame.shape
        # take the closest sub image given the more accurate coordinates
        x_norm,x2_norm,y_norm,y2_norm = self.coords_to_int()
        
        # coordinates are flipped all over the place. Fix by transpose
        self.frameCrop = np.copy(self.frame.T[x_norm:x2_norm,y_norm:y2_norm])
        
        self.ProcessCrop()

        return np.array([self.x,self.x2, self.y, self.y2]), img

    def ProcessCrop(self):

        # TODO: This stabilizes tracks a lot 
        # but might be useless with actual beads that are sharp
        # so test !
        #self.frameCrop = cv2.GaussianBlur(self.frameCrop,(3,3),0)
        ret3,binary = cv2.threshold(self.frameCrop,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        if binary is None:
            binary = np.zeros_like(self.frameCrop)
        else:
            binary = 255-binary
        
        self.binary = np.copy(binary)
        contours,_ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        radius = -1
        
        if contours:
            biggest = max(contours, key = cv2.contourArea)
            (x_,y_),radius = cv2.minEnclosingCircle(biggest)
            center = (int(x_),int(y_))
        else:
            biggest = np.zeros_like(binary)

        # old center
        old_x = (self.x2-self.x)/2
        old_y = (self.y2-self.y)/2

        # find center
        M = cv2.moments(biggest)
        if M["m00"]==0:
            cX = old_x
            cY = old_y
        else:
            cY = M["m10"] / M["m00"]
            cX = M["m01"] / M["m00"]

        # update global coordinates for this track
        y_int_error = self.y - self.yOrg
        x_int_error = self.x - self.xOrg
    
        shift_x = cX-old_x+x_int_error
        shift_y = cY-old_y+y_int_error

        self.x += shift_x
        self.x2 += shift_x
        self.y += shift_y 
        self.y2 += shift_y

        self.validateCoordinates()

        self.Dict["y"].append((self.y2+self.y)/2)
        self.Dict["x"].append((self.x2+self.x)/2)
        self.Dict["t"].append(self.t)


        self.xOrg = self.x
        self.yOrg = self.y    
        
        self.t += 1

    def saveData(self,path):
        # save dictionary to person_data.pkl file
        with open(os.path.join(path,"trackingData_{}.pickle".format(datetime.date.today())), 'wb') as fp:
            pickle.dump(self.Dict, fp)

    def validateCoordinates(self):
        # validate that coordinates are inside image
        x_max = self.w
        y_max = self.h
        # upper limit
        if self.x >= x_max:
            self.x = x_max-1
        if self.x2>= x_max:
            self.x2 = x_max-1
        if self.y>=y_max:
            self.y = y_max-1
        if self.y2>=y_max:
            self.y2 = y_max-1
        # lower limit
        if self.x<0:
            self.x = 0
        if self.x2<0:
            self.x2 = 0
        if self.y<0:
            self.y = 0
        if self.y2<0:
            self.y2 = 0

    def closeAll(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":

    videoPath = "//home.org.aalto.fi/lehtona6/data/Documents/data/recording.avi"

    track = tracker()
    track.main(videoPath)



    """
        #Incease birghtness if the video is dark (visualization)
        def increase_brightness(self, value=60):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] += value

        final_hsv = cv2.merge((h, s, v))
        self.frame = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        def displayFrame(self):
        #Draw boxes
        start_pos = (int(np.round(self.y)),int(np.round(self.x)))
        end_pos = (int(np.round(self.y2)),int(np.round(self.x2)))
        print(start_pos, end_pos)
        # display
        cv2.rectangle(self.frame, start_pos, end_pos, self.col, 5)
        cv2.imshow('win',self.frame)
        self.k = cv2.waitKey(1) & 0xFF
        if self.k == 27:
            self.closeAll()
            exit(0)

        def main(self, path):
        self.cap = cv2.VideoCapture(path)
        self.frameNum = 0
        while self.cap.isOpened():
            frame_exist, self.frame = self.cap.read()
            if not frame_exist:
                self.closeAll()
                break
            
            self.w,self.h,_ = self.frame.shape
            self.increase_brightness()

            imgOut = np.copy(self.frame)
            if self.frameNum == 0:
                self.chooseTarget()

                self.frame = imgOut
            
            
            cv2.namedWindow("win",cv2.WINDOW_NORMAL)
            self.frameCrop = np.copy(self.frame[self.x:self.x2,self.y:self.y2])
            
            self.ProcessCrop()
            self.frameNum +=1

        def chooseTarget(self):
        self.y = int(self.fBound[0][0][0])
        self.y2 = int(self.fBound[0][1][0])
        self.x = int(self.fBound[0][0][1])
        self.x2 = int(self.fBound[0][1][1])


    def click_events(self,event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            #if len(self.fBound) > 2:
            #    self.refPt = []
            #    self.fBound = []
            self.refPt = [(x, y)]
        elif event == cv2.EVENT_LBUTTONUP:
            self.refPt.append((x, y))
            self.fBound.append((self.refPt[0],self.refPt[1]))

            cv2.rectangle(self.frame, self.refPt[0], self.refPt[1], self.col, 4)
            cv2.imshow("win", self.frame)
        elif event == cv2.EVENT_MOUSEMOVE and flags == cv2.EVENT_FLAG_LBUTTON:
            clone = self.frame.copy()
            cv2.rectangle(clone, self.refPt[0], (x, y), self.col, 4)
            cv2.imshow("win", clone)
    """