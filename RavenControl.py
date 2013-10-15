################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import Leap, sys
import roslib; roslib.load_manifest("raven_2_teleop")
import rospy
import math
from numpy import *
from numpy.linalg import *
import tf
import tf.transformations as tft
from raven_2_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from raven_2_trajectory.srv import RecordTrajectory, RecordTrajectoryResponse
from optparse import OptionParser

from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

BASE_FRAME = '/0_link'
END_EFFECTOR_FRAME_PREFIX = '/tool_'
END_EFFECTOR_FRAME_SUFFIX = ['L','R']
SIDE_ACTIVE = [True,False]
SIDE_NAMES = ['L','R']
SIDE_NAMES_FRIENDLY = ['left','right']

LeapMotionFrames = []
FramesLock = threading.Lock()

class Listener(Leap.Listener):
    def on_init(self, controller):
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

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()


        FramesLock.acquire()
        if len(LeapMotionFrames) < 1000:
            LeapMotionFrames.append(frame)
        else:
            LeapMotionFrames = LeapMotionFrames[500:]
            LeapMotionFrames.append(frame)
        FramesLock.release()


        print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))

        if not frame.hands.empty:
            # Get the first hand
            hand = frame.hands[0]

            # Check if the hand has any fingers
            fingers = hand.fingers
            if not fingers.empty:
                # Calculate the hand's average finger tip position
                avg_pos = Leap.Vector()
                for finger in fingers:
                    avg_pos += finger.tip_position
                avg_pos /= len(fingers)
                print "Hand has %d fingers, average finger tip position: %s" % (
                      len(fingers), avg_pos)

            # Get the hand's sphere radius and palm position
            print "Hand sphere radius: %f mm, palm position: %s" % (
                  hand.sphere_radius, hand.palm_position)

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            print "Hand pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG)

            # Gestures
            for gesture in frame.gestures():
                if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                    circle = CircleGesture(gesture)

                    # Determine clock direction using the angle between the pointable and the circle normal
                    if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/4:
                        clockwiseness = "clockwise"
                    else:
                        clockwiseness = "counterclockwise"

                    # Calculate the angle swept since the last frame
                    swept_angle = 0
                    if circle.state != Leap.Gesture.STATE_START:
                        previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                        swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

                    print "Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                            gesture.id, self.state_string(gesture.state),
                            circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness)

                if gesture.type == Leap.Gesture.TYPE_SWIPE:
                    swipe = SwipeGesture(gesture)
                    print "Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
                            gesture.id, self.state_string(gesture.state),
                            swipe.position, swipe.direction, swipe.speed)

                if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
                    keytap = KeyTapGesture(gesture)
                    print "Key Tap id: %d, %s, position: %s, direction: %s" % (
                            gesture.id, self.state_string(gesture.state),
                            keytap.position, keytap.direction )

                if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
                    screentap = ScreenTapGesture(gesture)
                    print "Screen Tap id: %d, %s, position: %s, direction: %s" % (
                            gesture.id, self.state_string(gesture.state),
                            screentap.position, screentap.direction )

        if not (frame.hands.empty and frame.gestures().empty):
            print ""

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

class RavenController:
    def __init__(self, listener, scale=None, frame=None, relative_orientation=False, camera_frame=False):
        self.pub = rospy.Publisher('raven_command', RavenCommand)
        self.listener = listener
        self.scale = scale
        self.scale_increment = scale/20
        if frame:
            self.frame = frame
        else:
            self.frame = BASE_FRAME
        if camera_frame:
            pass
            #T1 = mat(array([[0,1,0,0],  [-1,0,0,0],  [0,0, 1,0], [0,0,0,1]]))
            #T2 = mat(array([[1,0,0,0],  [0,-1,0,0],  [0,0,-1,0], [0,0,0,1]]))
        else:
            self.T1 = mat(array([[0,1,0,0],  [-1,0,0,0],  [0,0, 1,0], [0,0,0,1]]))
            self.T2 = mat(eye(4)) 

        if SIDE_ACTIVE[0]:
            end_effector_frame = END_EFFECTOR_FRAME_PREFIX + END_EFFECTOR_FRAME_SUFFIX[0]
        else:
            end_effector_frame = END_EFFECTOR_FRAME_PREFIX + END_EFFECTOR_FRAME_SUFFIX[1]
        print "waiting for transform from {0} to {1}".format(BASE_FRAME, end_effector_frame)
        tries = 0
        while not rospy.is_shutdown():
            tries += 1
            #print "Try #%d" % tries
            try:
                self.listener.waitForTransform(BASE_FRAME, end_effector_frame, rospy.Time(0), rospy.Duration(5.0))
                break
            except tf.Exception, e:
                continue            
        self.LeapMotionListener = Listener()           
        controller = Leap.Controller()
        controller.add_listener(listener)                   
        while True:
            self.publishCommand()

    def calculateTransform(self, prevFrame, currFrame):
        # takes a previous leap motion frame and a current leapmotion frame and calculates the transform between them
        if not prevFrame.hands.empty: #FIXME WILL NEED TO GENERALIZE FOR TWO HANDS
            prevHand = prevFrame.hands[0]
        if not currFrame.hands.empty: 
            currHand = currFrame.hands[0]
        """prev_palm_pos = prevFrame.palm_position
        curr_palm_pos = currFrame.palm_position
        prev_palm_ori = (prevHand.normal.roll, prevHand.direction.pitch, prevHand.direction.yaw)
        curr_palm_ori = (currHand.normal.roll, currHand.direction.pitch, currHand.direction.yaw)"""
        translation = currHand.translation(prevFrame) #this is a Leap.Vector!!
        rot_matrix  = currHand.rotation(prevFrame) # this is a Leap.Matrix!!

    def publishCommand(self):
        if len(LeapMotionFrames)>0:
            frame = LeapMotionFrames[-1] #get the latest frame

            raven_command = RavenCommand()
            raven_command.header.stamp = rospy.Time.now()
            raven_command.header.frame_id = BASE_FRAME
            raven_command.controller = Constants.CONTROLLER_CARTESIAN_SPACE #FIXME: need to find where this is and stick it in the proper place
            active = [False, False]
            for i in xrange(2):
                if SIDE_ACTIVE[i]:
                    active[i] = True

            for i in xrange(2):
                arm_cmd = ArmCommand()
                tool_cmd = ToolCommand()
                tool_cmd.pose_option = ToolCommand.POSE_OFF

                try:
                        (trans,rot) = self.listener.lookupTransform(BASE_FRAME, END_EFFECTOR_FRAME_PREFIX + END_EFFECTOR_FRAME_SUFFIX[i], rospy.Time(0))
                    except (tf.LookupException, tf.ConnectivityException):
                        if i==0:
                            print "no transform for left arm!"
                        else:
                            print "no transform for right!"
                        return
                    xcur, ycur, zcur = trans[0],trans[1],trans[2]
                    xcmd, ycmd, zcmd = (paddle.transform.translation.x * self.scale,
                                        paddle.transform.translation.y * self.scale,
                                        paddle.transform.translation.z * self.scale)

                    self.xoffset, self.yoffset, self.zoffset = xcur-xcmd, ycur-ycmd, zcur-zcmd


def main():
    # Create a sample listener and controller
    listener = Listener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    sys.stdin.readline()

    # Remove the sample listener when done
    controller.remove_listener(listener)


if __name__ == "__main__":
    main()
