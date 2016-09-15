#############################
# BabyTracker.py
# Shahzore Qureshi
# 09/21/2016
# Sapient SNAP
#
# Program below tracks baby
# movements and sounds and
# triggers certain actions
# like lights or vibrations.
#############################


# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import imutils
import math
import RPi.GPIO as GPIO
from multiprocessing import Pool
from multiprocessing import Value
from ctypes import c_bool
import subprocess
import Adafruit_ADS1x15
import signal
import sys

######### START OF GLOBAL VARIABLES #########

# declare GPIO pins
PIN_LIGHTS = 23
PIN_VIBRATION_MOTOR = 24
PIN_MICROPHONE = 26

# declare camera-related objects
camera = None
rawCapture = None

# declare microphone-triggered flags
isBabyCryingGently = Value(c_bool, False)
isBabyCryingHysterically = Value(c_bool, False)

# create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()

# flag that is enabled when user presses Ctrl + C
hasProgramEnded = False

######### END OF GLOBAL VARIABLES #########

# create new class to represent a plot point
# this is useful when tracking the baby
class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

# initialize all GPIO pins
def setupGPIO():
	# setup GPIO pins
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(PIN_LIGHTS, GPIO.OUT)
	GPIO.setup(PIN_VIBRATION_MOTOR, GPIO.OUT)
	GPIO.setup(PIN_MICROPHONE, GPIO.IN)
	
	# keep motor and lights off when initializing
	GPIO.output(PIN_LIGHTS, GPIO.LOW)
	GPIO.output(PIN_VIBRATION_MOTOR, GPIO.HIGH)

# initialize all camera and tracking dependencies
def setupCamera():
	# initialize the camera as a global object
	global camera
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 64
	
	# grab a reference to the raw camera capture
	# and set the reference as global
	global rawCapture
	rawCapture = PiRGBArray(camera, size=(640, 480))

	# allow the camera to warmup
	time.sleep(0.1)
	
# start tracking the baby
def trackBaby():
	# declare the avg frame in the video stream
	avgFrame = None
	
	# declare avg velocity
	avgVelocity = 0
	
	# declare velocity counter (measures average sample size)
	velocityCounter = 0
	
	# declare max value for counter above
	velocityCounterMax = 4

	# declare previous frame average midpoint
	prevAvgMidPoint = None
	
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		# grab the raw NumPy array representing the image
		image = frame.array

		# add image effects that enhance the clarity of
		# any moving objects
		image = imutils.resize(image, width=500)
		gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)
		
		# if the average frame is None, initialize it
		# by copying the current image and ending the
		# current for loop iteration (so that we can
		# get the next frame)
		if avgFrame is None:
			avgFrame = gray.copy().astype("float")
			rawCapture.truncate(0)
			continue
		
		# accumulate the weighted average between the current frame and
		# previous frames, then compute the difference between the current
		# frame and running average
		cv2.accumulateWeighted(gray, avgFrame, 0.5)
		frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(avgFrame))
	 
		# threshold the delta image, dilate the thresholded image to fill
		# in holes, then find contours on thresholded image
		thresh = cv2.threshold(frameDelta, 5, 255,
			cv2.THRESH_BINARY)[1]
		thresh = cv2.dilate(thresh, None, iterations=2)
		(_, cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
	 
		# loop over the contours, which are shapes of moving objects.
		# also, keep track of the midpoints of each contour.
		midpoints = [];
		for c in cnts:
			# if the contour is too small, ignore it
			# and end the current for loop iteration
			if cv2.contourArea(c) < 1000:
				continue
	 
			# compute the bounding box for the contour, draw it on the frame,
			# and calculate the midpoint of the contour
			(x, y, w, h) = cv2.boundingRect(c)
			cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
			midX = x + w/2;
			midY = y + h/2;
			midpoints.append(Point(midX, midY));
		
		# if there are contours, find average midpoint
		# of all contours and calculate velocity of
		# recorded movement
		if midpoints:		
			# calculate average midpoint of all contours
			numOfMidpoints = len(midpoints)
			avgMidX = 0
			avgMidY = 0
			for p in midpoints:
				avgMidX += p.x
				avgMidY += p.y
			avgMidX /= numOfMidpoints
			avgMidY /= numOfMidpoints

			# use the class we declared at the beginning
			# of this program to organize the data better
			avgMidPoint = Point(avgMidX, avgMidY)
			
			# draw midpoint on current frame
			cv2.circle(image, (avgMidPoint.x, avgMidPoint.y), 3, (255, 0, 0), -1)
			
			# calculate velocity of average midpoint
			# by comparing current midpoint with previous
			# midpoint
			if prevAvgMidPoint is not None:
				diffX = abs(prevAvgMidPoint.x - avgMidPoint.x)
				diffY = abs(prevAvgMidPoint.y - avgMidPoint.y)
				velocity = math.sqrt(diffX * diffX + diffY * diffY)
				
				# add up average velocity
				avgVelocity += velocity
			
				# keep track of the number of velocites
				# that have been stored so far	
				velocityCounter = velocityCounter + 1
				print "velocityCounter: " + str(velocityCounter)

				# once we have the max number of velocites,
				# calculate the average velocity, trigger
				# a response for the baby, and reset the
				# average velocity, counter, and saved
				# image frame from camera
				if velocityCounter == velocityCounterMax:
					
					# calculate average velocity
					avgVelocity /= velocityCounter
					print "avg velocity: " + str(avgVelocity)
					
					# use velocity to determine what
					# action should be taken to soothe
					# the baby (ex. lights, sound)
					sootheBaby(avgVelocity)
					
					# reset values to zero
					avgVelocity = 0
					velocityCounter = 0
					avgFrame = None

			# keep track of midpoint so that it can
			# be compared to the next midpoint
			prevAvgMidPoint = avgMidPoint
			
		# show the camera feed via popup window.
		# comment the code below to increase
		# overall performance of the program.
		#
		# cv2.imshow("Frame", image)

		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)

		# if camera feed is shown, allow user to
		# close feed via the "q" key
		key = cv2.waitKey(1) & 0xFF

		# if the `q` key or the Ctrl + C combo is triggered,
		# terminate the program
		if key == ord("q") or hasProgramEnded:
			GPIO.cleanup()
			cv2.destroyAllWindows()
			camera.close()
			break

# trigger a certain action to soothe the baby 
# depending on the given velocity
def sootheBaby(velocity):
	# for demo purposes, ignore the velocity
	# and just trigger actions
	'''
	if velocity > 10 and velocity <= 50:
		playMusic()
	elif velocity > 50:
		playMotherSound()
		testVibrations()
	'''
	playMotherSound()
	testVibrations()

# trigger lights in a certain sequence	
def testLights():
	x = 0
	while x < 12:
		print("lights hi")
		GPIO.output(PIN_LIGHTS, GPIO.HIGH)
		time.sleep(0.5) 
		print("lights low")
		GPIO.output(PIN_LIGHTS, GPIO.LOW)
		time.sleep(0.5)
		x += 1

# trigger the mother's voice to calm
# the baby down
def playMotherSound():
	print("start mother sound")
	player = subprocess.Popen(["omxplayer", "-o", "local", "--vol", "800", "shush.wav"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	time.sleep(8)
	try:
		player.stdin.write("q")
	except IOError:
		print("music error")

# trigger music to calm the baby down
def playMusic():
	print("start music")
	player = subprocess.Popen(["omxplayer", "-o", "local", "--vol", "800", "sample.mp3"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	time.sleep(10)
	try:
		player.stdin.write("q")
	except IOError:
		print("music error")

# trigger vibrations in a certain sequence
def testVibrations():
	print("vib low")
	GPIO.output(PIN_VIBRATION_MOTOR, GPIO.LOW)
	time.sleep(5)
	print("vib hi")
	GPIO.output(PIN_VIBRATION_MOTOR, GPIO.HIGH)

# use the microphone to determine if the baby is
# making noises	
def trackSound():
	# grab global references of microphone-related flags
	# so that 'while' loop below can update them
	global isBabyCryingGently
	global isBabyCryingHysterically

	# sample the audio being recorded
	# by analyzing the sound peaks
	# and obtaining the amplitude
	# of the sound waves
	sample = 0
	sampleWindow = 150
	peakToPeakMin = 1898
	peakToPeakMax = 27699

	# data observations:
	## min value of adc.read_adc() = 12613
	## max value of adc.read_adc() = 29848
	## min value of peakToPeak = 1898
	## max value of peakToPeak = 27699

	while True:
		startMillis = int(round(time.time() * 1000))
		peakToPeak = 0
		signalMax = 0
		# signalMin = 1024
		signalMin = 99999

		while int(round(time.time() * 1000)) - startMillis < sampleWindow:
			sample = adc.read_adc(0, gain=2)
			#print "sample: " + str(sample)
			if sample > signalMax:
				signalMax = sample
			elif sample < signalMin:
				signalMin = sample

		peakToPeak = signalMax - signalMin
		print "peakToPeak: " + str(peakToPeak)
		time.sleep(0.2)
	
		# determine if baby is crying.
		# there are two levels of crying: gently or hysterically.	
		if peakToPeak > 650 and peakToPeak <= 1200:
			isBabyCryingGently.value = True
			isBabyCryingHysterically.value = False
			print("isBabyCryingGently: " + str(isBabyCryingGently.value))
			testLights()
		elif peakToPeak > 1200:
			isBabyCryingGently.value = False
			isBabyCryingHysterically.value = True
			print("isBabyCryingHysterically: " + str(isBabyCryingHysterically.value))
			testLights()



# handle Ctrl + C keypress by setting a flag to true.
# let the trackBaby() loop handle the destruction
# and deallocation of resources.
def signal_handler(signal, frame):
	print 'You pressed Ctrl + C'
	global hasProgramEnded
	hasProgramEnded = True

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
	setupGPIO()
	setupCamera()
	
	# start background audio tracking
	pool = Pool(processes=1)
	audioTracking = pool.apply_async(trackSound, [], callback=None)

	trackBaby()
	#trackSound()
	#testVibrations()
	#testVibrations()
	#testLights()
	#testLights()
	#testLights()
	#playMusic()
	#playMotherSound()

	GPIO.cleanup()
	cv2.destroyAllWindows()
	camera.close()
	sys.exit(0)
