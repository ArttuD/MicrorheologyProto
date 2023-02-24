from PyQt6 import QtWidgets, QtCore

from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys
import numpy as np

from NiDriver import niDevice

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__( *args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        
        
        self.setCentralWidget(self.graphWidget)

        #Make it pleasent
        self.graphWidget.setBackground("black")
        self.graphWidget.setTitle("Feedback and real", color="r", size = "15pt")

        styles = {"color": "#f00", "font-size": "20px"}
        self.graphWidget.setLabel("left", "Current [A]", **styles)
        self.graphWidget.setLabel("bottom", "Time [s]", **styles)
        #Add legend
        #Add grid
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setXRange(0, 10, padding=0, update = False)
        #self.graphWidget.enableAutoRange(axis=pg.ViewBox.XAxis, enable=0.5)
        self.time = np.zeros(100)
        self.target = np.zeros(100)
        self.measured = np.zeros(100)
        self.measuredB = np.zeros(100)
        self.data = None
        self.graphWidget.addLegend()
        self.dataLineTarget = self.graphWidget.plot(self.time, self.target, pen=pg.mkPen(color="red"), symbol='o', symbolSize=10, name = "target")
        self.dataLineMeasured = self.graphWidget.plot(self.time, self.measured, pen=pg.mkPen(color="blue"), symbol='s', symbolSize=10, name = "Measured current")
        self.dataLineMeasuredB = self.graphWidget.plot(self.time, self.measuredB, pen=pg.mkPen(color="green"), symbol='t', symbolSize=10, name = "Measured current from B")
        
        

        #self.timer = QtCore.QTimer()
        #self.timer.setInterval(100)
        #self.timer.timeout.connect(self.update_plot_data)
        #self.timer.start()

    def update_plot_data(self):
        self.time = np.roll(self.time,-1); self.time[-1] = self.data[0]
        self.target = np.roll(self.target,-1); self.target[-1] = self.data[1]
        self.measured = np.roll(self.measured,-1); self.measured[-1] = self.data[2]

        self.measuredB = np.roll(self.measuredB,-1); self.measuredB[-1] = self.data[3]
        
        self.dataLineTarget.setData(self.time, self.target)
        self.dataLineMeasured.setData(self.time, self.measured)
        self.dataLineMeasuredB.setData(self.time, self.measuredB)
        
        if self.data[0]>10:
            self.graphWidget.setXRange(self.data[0]-10, self.data[0]+10, padding=0)

    def receiveData(self,datas):
        self.data = datas
        self.update_plot_data()
        self.data = None

    def clean(self):
        self.graphWidget.clear()

def pymain(args):
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    driver = niDevice(args, w)
    running = driver.start()
    print("started driver", running)
    w.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    pymain()


