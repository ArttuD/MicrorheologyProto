import argparse
import sys
import os
from pyQt import *

import configparser
import logging


config = configparser.ConfigParser()

config_path = sys.argv[0]

# read specified config file?
if config_path.endswith('.ini'):
    c_path = os.path.join(os.getcwd(),'configs',config_path)
    if os.path.exists(c_path):
        config.read(c_path)
    else:
        raise NotADirectoryError(c_path)
else:
    # otherwise get the latest
    config.read(os.path.join(os.getcwd(),'configs','default.ini'))


parser = argparse.ArgumentParser(description='Microrheology test setup.')
parser.add_argument('--path', '-p', required = False, type = str, default = r'F:\test', help='Save path for all the files, creates folder etc')
parser.add_argument("--user", "-u", required=False, default = "DEFAULT",
                    help = "buffer size for sampling")
parser.add_argument("--buffer_size_cfg", "-b", required=False, default = None,
                    help = "buffer size for sampling")
parser.add_argument("--chans_in", "-c", required=False, default = None,
                    help = "Number of sensors for sampling")
parser.add_argument("--time", "-t", required=False, default =  None,
                    help = "Total time of the measurement")
parser.add_argument("--exposure", "-e", required=False, default = None,
                    help = "Camera's exposure time in ms")
parser.add_argument("--framerate", "-f", required=False, default =  None,
                    help = "Cameras framerate in fps")
parser.add_argument("--FirstResis", "-r", required=False, default = None,
                    help = "Resistance of current sensor 1")
parser.add_argument("--samplingFreq", "-sf", required=False, default = None,
                    help = "Sampling frequency")      
parser.add_argument("--conversionFactor", required=False, default = None,
                    help = "Convert B to i")    
parser.add_argument('-d', '--debug',
                    help="Print lots of debugging statements",
                    action="store_const", dest="loglevel",
                    const=logging.DEBUG,
                    default=logging.WARNING)
parser.add_argument('-v', '--verbose',help="Be verbose",
                    action="store_const", dest="loglevel",
                    const=logging.INFO)
args = parser.parse_args()


args.buffer_size_cfg = int(config[args.user]["buffer_size_cfg"] )
args.chans_in = int(config[args.user]["chans_in"] )
args.time = int(config[args.user]["time"] )
args.exposure = int(config[args.user]["exposure"]) 
args.framerate =int( config[args.user]["framerate"]) 
args.FirstResis = float(config[args.user]["FirstResis"]) 
args.samplingFreq = int( config[args.user]["samplingFreq"])
args.conversionFactor = float( config[args.user]["conversionFactor"])

isExist = os.path.exists(args.path)

if not isExist:
   # Create a new directory because it does not exist
   os.makedirs(args.path)
   print("The new directory is created: ", args.path )

#launch graph

logging.basicConfig(
                format='%(levelname)s:%(message)s',
                level=args.loglevel)

logging.info(f'Using: {args.user} configuration')

pymain(args, logging)