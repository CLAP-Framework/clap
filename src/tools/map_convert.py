#!/usr/bin/python2
import sys
import argparse
import utm
import numpy as np

# help : convert -origin 
arg_parser = argparse.ArgumentParser()
arg_parser.add_argument('--origin', required=True, help='origin point, default 0.0,0.0')
arg_parser.add_argument('--input',  required=True, help='gps track txt file path')
arg_parser.add_argument('--output', required=True, help='points file path converted base utm')

args = arg_parser.parse_args()

converted_dat = []
data = np.loadtxt(args.input)
# 
# origin = (428191,4417667)
origins = args.origin.split(',')
ox, oy = float(origins[0]), float(origins[1])

for wp in data:
    point = utm.from_latlon(wp[1], wp[0])
    # lon lat x y z w
    converted_dat.append((point[0] - ox, point[1] - oy, wp[3], wp[4], wp[5], wp[2]))

print(converted_dat)
np.savetxt(args.output, converted_dat, fmt="%.8f,%.8f,%.8f,%.8f,%.8f,%.8f", delimiter="\n")
