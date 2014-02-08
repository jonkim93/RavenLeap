<<<<<<< HEAD

#TODO: download TFX
#TODO: make sure function calls are correct
#TODO: figure out pose type and how to change it translationally
#TODO: check joint limits and make sure they're not being hit

#====== LEAP =============#
import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

#====== GENERAL ==========#
import math
from numpy import *
from numpy.linalg import *
from optparse import OptionParser

#====== ROS ==============#
import tf
import tf.transformations as tft
from RavenKin import *
#import roslib; roslib.load_manifest("raven_2_teleop")
#import rospy
#from raven_2_msgs.msg import *
#from geometry_msgs.msg import Pose, Point, Quaternion
#from raven_2_trajectory.srv import RecordTrajectory, RecordTrajectoryResponse

#====== OPENRAVE =========#
import openravepy as rave



BASE_FRAME = '/0_link'
END_EFFECTOR_FRAME_PREFIX = '/tool_'
END_EFFECTOR_FRAME_SUFFIX = ['L','R']
SIDE_ACTIVE = [True,False]
SIDE_NAMES = ['L','R']
SIDE_NAMES_FRIENDLY = ['left','right']

#========================= CONSTANTS ============================#
MODE = "SIM" # or "REAL"
ARM = "ONE_ARM" # or "TWO_ARM"
MODEL_NAME = "myRaven.xml" 

JOINTS_ARRAY_INDICES = [SHOULDER, ELBOW, Z_INS, TOOL_ROT, WRIST, GRASP1, GRASP2, YAW, GRASP]

prevFrame = None
currFrame = None
FramesLock = threading.Lock()

#========================= LEAP MOTION LISTENER CLASS ======================================================#
class Listener(Leap.Listener):
=======
################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

MODES = ["PAUSE", "MASTER-SLAVE", "AUTONOMOUS"]
MODE_INDEX = 1

def changeMode(direction):
    if direction == "left": #FIXME: not sure if this is correct notation for swipe directions
        MODE_INDEX = (MODE_INDEX+1)%3
    elif direction == "right": #FIXME: not sure if this is correct notation for swipe directions
        MODE_INDEX = (MODE_INDEX-1)%3
    print MODES[MODE_INDEX]

def sendError():
    print "ERROR"

def sendRavenCommand(command, fingers):
    if MODES[MODE_INDEX]=="MASTER-SLAVE":
        if len(fingers)==2:

        else:
            print "ERROR: USE TWO FINGERS WHEN COMMANDING RAVEN"


class SampleListener(Leap.Listener):
>>>>>>> test
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

<<<<<<< HEAD
=======
    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

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
                    changeMode(swipe.direction)

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

>>>>>>> test
    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

<<<<<<< HEAD
    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()
        FramesLock.acquire()
        if prevFrame == None:
            currFrame = frame
        else:
            prevFrame = currFrame
            currFrame = frame
        FramesLock.release()

    
#========================= RAVEN CONTROLLER CLASS ==========================================================#
class RavenController:
    def __init__(self, scale=0.1, frame=None, relative_orientation=False, camera_frame=False):
        self.raven_pub = rospy.Publisher('raven_command', RavenCommand)
        self.scale = scale
        self.scale_increment = scale/20
        
        self.LeapMotionListener = Listener()           
        controller = Leap.Controller()
        controller.add_listener(listener)     

        """if frame:
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
                continue            """     

    def run(self):
        if MODE == "REAL":
            while True:
                raven_command = self.getRavenCommand()  # start continually publishing raven commands based on input from the leapmotion
                self.raven_pub.publish(raven_command)
        elif MODE == "SIM":
            self.configureOREnv()
            while True:
                joints = self.getORCommand()
                self.publishORCommand(joints)

        # Remove the sample listener when done
        controller.remove_listener(listener)   

    #========================== OPEN RAVE COMMANDS ================================#

    def configureOREnv(self):
        env = rave.Environment()
        env.Load('myRaven.xml')
        self.robot = robot.GetRobots()[0]
        self.manipulators = robot.GetManipulators()
        manip = manipulators[0]
        manipIndices = manip.GetArmIndices()

        # NOT SURE ABOUT FUNCTION CALL NAMES 
        allJoints = robot.GetJoints()
        jointIndices = manip.GetArmIndices()
        print "JOINTINDICES: "+jointIndices
        manipJoints = [allJoints[i] for i in jointIndices]
        manipJointValues = [manipJoint.GetDOFValue() for manipJoint in manipJoints]
        print "MANIPJOINTVALUES: "+manipJointValues
        #FIXME: look at this
        self.prevPose = fwdArmKin(0, manipJointValues) 

    def publishORCommand(self, joints):
        #set joint values
        self.robot.SetJointsValues(joints, JOINTS_ARRAY_INDICES)

    def getORCommand(self):
        FramesLock.acquire()
        p = prevFrame
        c = currFrame
        FramesLock.release()
        prevPose = self.prevPose
        print "prevPose: "+str(prevPose)
        if p != None and c != None:
            translation = calculateTransform(p, c, arm)
            #BASED OFF THIS TRANSLATION, WE NEED TO DIRECTLY CHANGE THE POSE SOMEHOW. HOWWW???
            dx, dy, dz = (translation.x*self.scale, 
                        translation.y*self.scale, 
                        translation.z*self.scale)

    

    #========================== ROBOT/ROS COMMANDS =================================#

    def getRavenCommand(self):
        """
        Calculates a raven command based off of LeapMotion frames
        """
        raven_command = RavenCommand()
        raven_command.header.stamp = rospy.Time.now()
        raven_command.header.frame_id = BASE_FRAME
                
        raven_command.controller = Constants.CONTROLLER_CARTESIAN_SPACE
        leftArmCommand = self.getArmCommand(0)
        rightArmCommand = self.getArmCommand(1)
        raven_command.arm_names = ['left', 'right']
        raven_command.arms = [leftArmCommand, rightArmCommand]

        #FIXME: need to account for pedal down and up in leapmotion commands
        raven_command.pedal_down = True
        return raven_command

    def getArmCommand(self, arm):
        """
        Given an arm index, calculates and returns the proper arm command based off of the LeapMotion frames 
        """
        FramesLock.acquire()
        p = prevFrame
        c = currFrame
        FramesLock.release()
        if p != None and c != None:
            translation, rot_matrix = calculateTransform(p, c, arm)
            arm_cmd = ArmCommand()
            tool_cmd = ToolCommand()
            tool_cmd.pose_option = ToolCommand.POSE_OFF

            try:
                (trans, rot) = self.listener.lookupTransform(BASE_FRAME, END_EFFECTOR_FRAME_PREFIX+END_EFFECTOR_FRAME_SUFFIX[i], rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException):
                if i==0:
                    print "no transform for left arm!"
                else:
                    print "no transform for right!"
                return
            xx, yy, zz, ww = (1, 0, 0, 0)  #FIXME: identity quaternion for now; will replace once we start focusing on 
            print "dx: "+dx+"  dy: "+dy+"  dz: "+dz+"\n"

            #FIXME: this stuff is probably wrong
            dx, dy, dz = (translation.x*self.scale, 
                        translation.y*self.scale, 
                        translation.z*self.scale)
            p = mat(array([dx,dy,dz,1])).transpose()
            T1 = mat(array([[0,1,0,0],  [-1,0,0,0],  [0,0, 1,0], [0,0,0,1]]))
            p_t = array(T1 * p)[0:3].flatten().tolist()

            qmat = T1 * mat(tft.quaternion_matrix(array([xx,yy,zz,ww])))
            q_t = array(tft.quaternion_from_matrix(qmat)).flatten().tolist()
            tool_cmd.pose_option = ToolCommand.POSE_RELATIVE #FIXME: this sets the toolcommand to relative pose commands

            tool_cmd.pose = Pose(Point(*p_t),Quaternion(*q_t))

            #FIXME: will need to add in grip commands later

            arm_cmd.tool_command = tool_cmd
            return arm_cmd
        else:
            if currFrame == None:
                print "currFrame equals none"
            if prevFrame == None:
                print "prevFrame equals none"
            return

#========== HELPER FUNCTIONS ==========#
def calculateTransform(prev, curr, index):
    """
    Static helper method for calculating the transform between two leapmotion frames 

    returns translation, rot_matrix of types Leap.Vector and Leap.Matrix
    """
    if not prev.hands.empty: #FIXME WILL NEED TO GENERALIZE FOR TWO HANDS
        prevHand = prev.hands[index]
    if not curr.hands.empty: 
        currHand = curr.hands[index]
    """prev_palm_pos = prev.palm_position
    curr_palm_pos = curr.palm_position
    prev_palm_ori = (prevHand.normal.roll, prevHand.direction.pitch, prevHand.direction.yaw)
    curr_palm_ori = (currHand.normal.roll, currHand.direction.pitch, currHand.direction.yaw)"""
    translation = currHand.translation(prev) #this is a Leap.Vector!!
    rot_matrix  = currHand.rotation(prev) # this is a Leap.Matrix!!
    return translation, rot_matrix    

#================= MAIN ================#
def main():
    rc = RavenController()
    rc.run()
=======
def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    sys.stdin.readline()

    # Remove the sample listener when done
    controller.remove_listener(listener)

>>>>>>> test

if __name__ == "__main__":
    main()
