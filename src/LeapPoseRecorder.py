#!/usr/bin/env python

import Leap
import sys
import getopt
import csv
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

class Listener(Leap.Listener):
    def on_init(self, controller):
    	self.frames = []
    	self.data = []
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"
        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

    def on_frame(self, controller):
		# Get the most recent frame and report some basic information
		frame = controller.frame()
		self.frames.append(frame)
		if len(frame.hands) != 0:
			print str(frame.timestamp)+": "+str(frame.hands[0].palm_position)
			self.data.append((frame.timestamp, frame.hands[0].palm_position, frame.hands[0].palm_normal, frame.hands[0].palm_position.x, frame.hands[0].palm_position.y, frame.hands[0].palm_position.z))
		else:
			self.data.append(("null", "null", "null", "null", "null", "null"))

def startRecording(name):
	listener = Listener()
	controller = Leap.Controller()
	print "\tcontroller initalized"     # Have the sample listener receive events from the controller
	controller.add_listener(listener)   # Keep this process running until Enter is pressed
	print "Press Enter to quit..."
	sys.stdin.readline()    
	data = listener.data
	with open(name+'.csv', 'wb') as csvfile:
	    datawriter = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
	    datawriter.writerow(("TimeStamp", "Palm Position", "Palm Normal", "x", "y", "z"))
	    for datum in data:
	    	datawriter.writerow(datum)
	controller.remove_listener(listener) # Remove the sample listener when done

def main(argv):
	name = "trials"
	try:
	    opts, args = getopt.getopt(argv,"hn:", ["name="])
	except getopt.GetoptError:
	    print 'LeapPoserecorder.py -n <name>'
	    sys.exit(2)
	for opt, arg in opts:
		if opt=="-h":
			print 'LeapPoserecorder.py -n <name>'
		elif opt == "-n":
			name = arg 
	startRecording(name)



if __name__=='__main__':
    main(sys.argv[1:])