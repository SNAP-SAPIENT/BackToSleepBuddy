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
import serial

######### START OF GLOBAL VARIABLES #########

# initialize Arduino board communication
arduino = serial.Serial('/dev/ttyACM0', 9600)

# declare GPIO pins
PIN_LIGHTS = 23
PIN_VIBRATION_MOTOR = 22
PIN_MICROPHONE = 26

# declare camera-related objects
camera = None
rawCapture = None

# declare microphone-triggered flags
isBabyCryingGently = Value(c_bool, False)
isBabyCryingHysterically = Value(c_bool, False)

######### END OF GLOBAL VARIABLES #########

# create new class to represent a plot point
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
	GPIO.output(PIN_VIBRATION_MOTOR, GPIO.HIGH)
	GPIO.output(PIN_LIGHTS, GPIO.LOW)

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
	velocityCounterMax = 20

	# declare previous frame average midpoint
	prevAvgMidPoint = None
	
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):		
		# grab the raw NumPy array representing the image, then initialize the timestamp
		# and occupied/unoccupied text
		image = frame.array

		image = imutils.resize(image, width=500)
		gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray, (21, 21), 0)
		
		# if the average frame is None, initialize it
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
	 
		# loop over the contours
		midpoints = [];
		for c in cnts:
			# if the contour is too small, ignore it
			if cv2.contourArea(c) < 1000:
				continue
	 
			# compute the bounding box for the contour, draw it on the frame,
			# and update the text
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
				
				# get microphone sound volume
				#~ volume = arduino.readline().rstrip()
				#~ if volume == "" or volume == " ":
					#~ volume = "0"
				
				# convert volume into an integer
				#~ volume = int(volume)
				#~ print "volume: " + str(volume)
				
				# add up average volume
				# avgVolume += volume
				 
				# find max volume
				#~ if avgVolume < volume:
					#~ avgVolume = volume
				
				velocityCounter = velocityCounter + 1
				print "velocityCounter: " + str(velocityCounter)
				
				if velocityCounter == velocityCounterMax:
					
					# calculate average velocity
					avgVelocity /= velocityCounter
					print "avg velocity: " + str(avgVelocity)
					
					# calculate average volume
					# avgVolume /= velocityVolumeCounter
					#~ print "avg volume: " + str(avgVolume)
					
					# use velocity
					# to determine how to calm the
					# baby down
					sootheBaby(avgVelocity)
					
					# reset values to zero
					avgVelocity = 0
					velocityCounter = 0
				
			prevAvgMidPoint = avgMidPoint
			
		
		cv2.imshow("Frame", image)
		key = cv2.waitKey(1) & 0xFF

		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)

		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			GPIO.cleanup()
			cv2.destroyAllWindows()
			camera.close()
			break

def sootheBaby(velocity):
	global isBabyCryingGently
	global isBabyCryingHysterically
	
	if velocity > 25 and velocity <= 100:
		playMusic()
		if isBabyCryingGently.value:
			testVibrations()
		elif isBabyCryingHysterically.value:
			testVibrations()
			testLights()
			testLights()
			testLights()
			testLights()
	elif velocity > 100:
		playMusic()
		testVibrations()
		if isBabyCryingGently.value:
			testLights()
			testLights()
			testLights()
			testLights()
	
	isBabyCryingGently.value = False
	isBabyCryingHysterically.value = False
	
def testLights():
	print("lights hi")
	GPIO.output(PIN_LIGHTS, GPIO.HIGH)
	time.sleep(0.5) 
	print("lights low")
	GPIO.output(PIN_LIGHTS, GPIO.LOW)
	time.sleep(0.5)

def playMusic():
	print("start music")
	player = subprocess.Popen(["omxplayer", "-o", "local", "--vol", "-500", "shush.wav"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	time.sleep(8)
	try:
		player.stdin.write("q")
	except IOError:
		print("music error")
	
def testVibrations():
	print("vib low")
	GPIO.output(PIN_VIBRATION_MOTOR, GPIO.LOW)
	time.sleep(0.5)
	print("vib hi")
	GPIO.output(PIN_VIBRATION_MOTOR, GPIO.HIGH)
	time.sleep(0.5)
	
def trackSound():
	avgVolume = 0
	
	# declare volume counter (measures average sample size)
	volumeCounter = 0
	
	# declare max value for counter above
	volumeCounterMax = 5
	
	# grab global references of microphone-related flags
	# so that 'while' loop below can update them
	global isBabyCryingGently
	global isBabyCryingHysterically
	
	while True:
		volume = arduino.readline().rstrip()
		if volume == "" or volume == " ":
			volume = "0"
			
		# convert volume into an integer
		volume = int(volume)
		# print "volume: " + str(volume)
		
		avgVolume += volume
		volumeCounter = volumeCounter + 1
		
		if volumeCounter == volumeCounterMax:
			# calculate average volume
			avgVolume /= volumeCounter
			print "avg volume: " + str(avgVolume)
			
			if avgVolume > 15 and avgVolume <= 30:
				if isBabyCryingHysterically.value is False:
					isBabyCryingGently.value = True
					print("isBabyCryingGently: " + str(isBabyCryingGently.value));
			elif avgVolume > 30:
				isBabyCryingGently.value = False
				isBabyCryingHysterically.value = True
				print("isBabyCryingHysterically: " + str(isBabyCryingHysterically.value));
			
			avgVolume = 0
			volumeCounter = 0
		
	
if __name__ == '__main__':
	setupGPIO()
	setupCamera()
	
	# start background audio tracking
	pool = Pool(processes=1)
	audioTracking = pool.apply_async(trackSound, [], callback=None)
	
	#trackSound()
	trackBaby()
