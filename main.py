import argparse
import sys
import os
from pyQt_video import *
from NiDriver import *

parser = argparse.ArgumentParser(description='Microrheology test setup.')
parser.add_argument('--path', '-p', required = True,
                    help='Save path for all the files, creates folder etc')
parser.add_argument("--buffer_size_cfg", "-b", required=False, default = 100,
                    help = "buffer size for sampling")
parser.add_argument("--chans_in", "-c", required=False, default = 2,
                    help = "buffer size for sampling")
parser.add_argument("--time", "-t", required=False, default = 60,
                    help = "Total time of the measurement")

args = parser.parse_args()

isExist = os.path.exists(args.path)
if not isExist:
   # Create a new directory because it does not exist
   os.makedirs(args.path)
   print("The new directory is created: ", args.path )

#launch graph
pymain(args)