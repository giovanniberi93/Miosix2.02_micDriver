#!/usr/bin/python
#
# This script plots vectors received via serial as couples of 
# indexes and values saparated by a semicolon. Each vector is
# separated by the string "new"
#
# Usage: plot.py device baudrate
#
# Esample: ./plot.py /dev/ttyUSB0 19200
#

import sys, serial, re
import matplotlib.pyplot as plt

plt.ion() #Interactive mode for plots
beginSignal = "new"
usage = """Usage: plot.py device baudrate

plot.py /dev/ttyUSB0 19200""";

if len(sys.argv) < 3:
	print("Too few arguments")
	print(usage)
	exit( 1 )
ser = serial.Serial(str(sys.argv[1]), int(sys.argv[2])) #no timeout specified, careful with readline: http://pyserial.sourceforge.net/shortintro.html
if ser < 1:
	print("Error opening serial port")
	print(usage)
	exit( 1 )

pattern = re.compile("(-)?\d+(.\d+)?:(-)?\d+(.\d+)?");
sample = ""
while sample != beginSignal: #wait for the begin signal
	sample = ser.readline().split("\r\n")[0]
	#print("#", sample) #prints the discarded lines before "new"
print(sample)
while True:
	FFT = {}
	sample = ser.readline()
	while sample != beginSignal:
		sample = ser.readline().split("\r\n")[0]
		# print("sample is $", sample)
		match = pattern.match(sample) 
		if match != None:
			print "dentro"
			sample = match.group()
			key, val = sample.split(":")
			FFT[float(key)] = float(val)
		else :
			print "# Regular expression error: ",sample 
	#print "++++   New Stuff   +++++"
	#for key in FFT:
	#	print key, ":", FFT[key]
	#print "++++++++++++++++++++++++"
	plt.cla()
	plt.plot(FFT.keys(),FFT.values(),)
	plt.draw()
