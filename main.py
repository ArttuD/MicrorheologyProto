import argparse
import sys
import os
from pyQt import *

parser = argparse.ArgumentParser(description='Microrheology test setup.')
parser.add_argument('--path', '-p', required = False, type = str, default = r'C:\Users\Asentaja\Git\MicrorheologyProto\test', help='Save path for all the files, creates folder etc')
parser.add_argument("--buffer_size_cfg", "-b", required=False, default = 100,
                    help = "buffer size for sampling")
parser.add_argument("--chans_in", "-c", required=False, default = 2,
                    help = "Number of sensors for sampling")
parser.add_argument("--time", "-t", required=False, default = 2*60,
                    help = "Total time of the measurement")
parser.add_argument("--exposure", "-e", required=False, default = 25,
                    help = "Camera's exposure time in ms")
parser.add_argument("--framerate", "-f", required=False, default = 40,
                    help = "Cameras framerate in fps")
parser.add_argument("--frameCount", "-fc", required=False, default = 40*120,
                    help = "Number of recorded images")
parser.add_argument("--FirstResis", "-r", required=False, default = 0.14,
                    help = "Resistance of current sensor 1")
parser.add_argument("--samplingFreq", "-sf", required=False, default = 10000,
                    help = "Sampling frequency")                    

args = parser.parse_args()
isExist = os.path.exists(args.path)

if not isExist:
   # Create a new directory because it does not exist
   os.makedirs(args.path)
   print("The new directory is created: ", args.path )

#launch graph
pymain(args)