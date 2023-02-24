import numpy as np
import cv2


class tracker():

    def __init__(self):
        self.frame = None
        self.refPt = []
        self.fBound = []
        self.w = None
        self.h = None
        self.k = None
        self.x = None
        self.x2 = None
        self.y = None
        self.y2 = None
        self.viz = True
        self.cap = None
        self.col = (0, 255, 0)
        self.Dict = {"x":[],"y":[],"t":[]}
        self.t = 0

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

    def chooseTarget(self):
        cv2.namedWindow("win",cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("win", self.click_events)
        cv2.imshow("win", self.frame)
        self.k = cv2.waitKey(0) & 0xFF

        cv2.destroyAllWindows()
        self.y = int(self.fBound[0][0][0])
        self.y2 = int(self.fBound[0][1][0])
        self.x = int(self.fBound[0][0][1])
        self.x2 = int(self.fBound[0][1][1])

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

    def ProcessCrop(self):
        self.frameCrop = cv2.cvtColor(self.frameCrop, cv2.COLOR_BGR2GRAY)
        ret3,binary = cv2.threshold(self.frameCrop,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        if binary is None:
            binary = np.zeros_like(self.frameCrop)
        else:
            binary = 255-binary
        
        contours,_ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        radius = -1
        if contours:
            biggest = max(contours, key = cv2.contourArea)
            #(x_,y_),radius = cv2.minEnclosingCircle(biggest)
            #center = (int(x_),int(y_))
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
        shift_x = cX-old_x
        shift_y = cY-old_y
        self.x += shift_x
        self.x2 += shift_x
        self.y += shift_y 
        self.y2 += shift_y
        self.validateCoordinates()
        if self.viz:
            self.displayFrame()
        self.Dict["y"].append((self.y2+self.y)/2)
        self.Dict["x"].append((self.x2+self.x)/2)
        self.Dict["t"].append(self.t)
        self.t += 1
        

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
        self.x = int(self.x)
        self.x2 = int(self.x2)
        self.y = int(self.y)
        self.y2 = int(self.y2)


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

    def closeAll(self):
        self.cap.release()
        cv2.destroyAllWindows()

    #Incease birghtness if the video is dark (visualization)
    def increase_brightness(self, value=60):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] += value

        final_hsv = cv2.merge((h, s, v))
        self.frame = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)


if __name__ == "__main__":

    videoPath = "//home.org.aalto.fi/lehtona6/data/Documents/data/recording.avi"

    track = tracker()
    track.main(videoPath)