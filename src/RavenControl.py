#!/usr/bin/env python

# NOTES:
# created by Jonathan Kim (Berkeley Automation Lab) 2014

# imports from Constants.py, OR_RavenController.py, ROS_RavenController.py, General_Helper.py

#====== LEAP =============#
import Leap

from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

#====== GENERAL ==========#
import sys
#from select import select
import getopt
import math
import numpy as np

import tty
import termios

from numpy import *
from numpy.linalg import *
from optparse import OptionParser

import pygame 
from pygame.locals import *

#import pygame
#import IPython
#import getch

#====== CUSTOM SCRIPTS ===#
from RavenKin import *
from Constants import *
from RavenControllers import *

#====== GLOBAL =========#
grip_type = "h"  # or can be "t"
operation_mode = "s"  # or can be "r"

"""
def check_clutch():
    def isData():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
    clutch_down = False
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        if isData():
            c = sys.stdin.read(1)
            print c 
            if c == '\x1b':         # x1b is ESC
                return
            if c == ' ':
                print "space"
                clutch_down = True                   
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return clutch_down
"""




#========================= LEAP MOTION LISTENER CLASS ======================================================#
class Listener(Leap.Listener):
    def isData(self):
        import select
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def consoleBased(self):
        #this works but you need to have your cursor in the console
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while 1:
                if self.isData():
                    c = sys.stdin.read(1)
                    #print c 
                    if c == '\x1b':         # x1b is ESC
                        break
                    if c == ' ':
                        print "space"
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def pygameBased(self):
        pygame.init()
        screen = pygame.display.set_mode( (320,240) )
        pygame.display.set_caption('Clutch Pedal Window')
        screen.fill((159, 182, 205))
        done = False
        while not done:
            pygame.event.pump()
            keys = pygame.key.get_pressed()
            #print keys
            if keys[K_ESCAPE]:
                done = True
            if keys[K_SPACE]:
                print "active"
                self.clutch_down = True
            else:
                self.clutch_down = False 
            
    

    def on_init(self, controller):
        print "Initialized"
        self.prevActiveFrameCounter = 0
        self.prevFrame = None
        self.currFrame = None
        self.clutch_down = False
        if operation_mode == "s" or operation_mode == "S":
            self.rc = OR_RavenController(grip_type)
        elif operation_mode == "r" or operation_mode == "R":
            self.rc = ROS_RavenController(grip_type)
        #self.consoleBased()
        #self.pygameBased()


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
        active = self.check_active(frame)
        grip, tipDistance = self.check_grip(frame)
        self.rc.updateActive(active)
        self.prevFrame = self.currFrame
        self.currFrame = frame
        if self.clutch_down:
            self.rc.run(self.prevFrame, self.currFrame, grip, tipDistance) 
        

    def check_grip(self, frame):
        if grip_type == "t":
            if not frame.hands.empty:
                hand = frame.hands[0]  # Get the first hand
                fingers = hand.fingers # Check if the hand has any fingers

                if len(fingers) == 2:
                    pos0 = fingers[0].tip_position
                    pos1 = fingers[1].tip_position
                    distanceBetweenTips = math.sqrt((pos0[0]-pos1[0])**2 + (pos0[1]-pos1[1])**2 + (pos0[2]-pos1[2])**2)
                    #print "distance between tips: ", distanceBetweenTips
                    return False, distanceBetweenTips
                elif len(fingers) == 1:
                    return True, 0
                else:
                    return False, -10
            return False, -10
        elif grip_type == "h":
            if not frame.hands.empty:
                hand = frame.hands[0]

                #print "HAND SPHERE RADIUS: ", hand.sphere_radius
                return False, hand.sphere_radius
            return False, -10

    def check_active(self, frame):
        if grip_type == "t":
            if not frame.hands.empty:
                hand = frame.hands[0]
                fingers = hand.fingers
                #print "ACTIVE FRAME COUNTER"
                #print self.prevActiveFrameCounter
                if len(fingers) <= 2:
                    self.prevActiveFrameCounter += 1
                    if self.prevActiveFrameCounter > ACTIVE_THRESHOLD:
                        return True
                    else:
                        return False
                else:
                    self.prevActiveFrameCounter = 0
                    return False
            return False
        elif grip_type == "h":
            return True


#================= MAIN ================#
def initialize(grip_option, mode_option):
    global grip_type
    grip_type = grip_option
    global operation_mode
    operation_mode = mode_option 
    
    # Create a sample listener and controller
    print "initializing . . ."

    listener = Listener()
    controller = Leap.Controller()

    print "\tcontroller initalized"     # Have the sample listener receive events from the controller

    controller.add_listener(listener)   # Keep this process running until Enter is pressed

    listener.pygameBased()
    print "Press Enter to quit..."
    sys.stdin.readline()
    
    controller.remove_listener(listener) # Remove the sample listener when done



def main(argv):
    try:
        opts, args = getopt.getopt(argv,"hg:m:",["grip option=", "mode="])
    except getopt.GetoptError:
        print 'RavenControl.py -g <grip option> -m <mode>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'RavenControl.py -g <grip option=(two_finger,t,whole_hand,h)> -m <mode=(SIM, S, ROBOT, R)>'
            sys.exit()
        elif opt in ("-g", "-grip"):
            grip_option = arg
        elif opt in ("-m", "-mode"):
            mode_option = arg 
    print 'Grip option is: ' + str(grip_option)
    print 'Mode option is: ' + str(mode_option)

    if mode_option in ("SIM", "S", "s", "sim"):
        mode_option = "s"
    elif mode_option in ("ROBOT", "R", "r", "robot"):
        mode_option = "r"

    if grip_option in ("two_finger", "t"):
        grip_option = "t"
    elif grip_option in ("whole_hand", "h"):
        grip_option = "h"
    else:
        print "Error: no such option exists; choose among (two_finger, t, whole_hand, h)"
        return 

    initialize(grip_option, mode_option)



if __name__=='__main__':
    main(sys.argv[1:])

