#!/usr/bin/env python

#TODO: make dictionary mapping from OR joints to ROS joints and vice versa
#TODO: check joint limits
#TODO: check raven_2/raven_2_trajectory/raven_planner.py
#TODO: something wrong with the pose calculation from joint angles . .. 

#====== LEAP =============#
import Leap, sys
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

#====== GENERAL ==========#
import math
import numpy as np 
from numpy import *
from numpy.linalg import *
from optparse import OptionParser
import threading
import IPython

#====== ROS ==============#
#import tf
#import tf.transformations as tft
from RavenKin import *
import tfx
#import roslib; roslib.load_manifest("raven_2_teleop")
#import rospy
#from raven_2_msgs.msg import *
#from geometry_msgs.msg import Pose, Point, Quaternion
#from raven_2_trajectory.srv import RecordTrajectory, RecordTrajectoryResponse

#====== OPENRAVE =========#
import openravepy as rave

"""

BASE_FRAME = '/0_link'
END_EFFECTOR_FRAME_PREFIX = '/tool_'
END_EFFECTOR_FRAME_SUFFIX = ['L','R']
SIDE_ACTIVE = [True,False]
SIDE_NAMES = ['L','R']
SIDE_NAMES_FRIENDLY = ['left','right']

"""

#========================= CONSTANTS ============================#
MODE = "SIM" # or "REAL"
ARM = "ONE_ARM" # or "TWO_ARM"
MODEL_NAME = "myRaven.xml" 
DEBUG = True

X_SCALE=0.0003
Y_SCALE=0.0003
Z_SCALE=0.0006
SHOULDER   =0
ELBOW      =1
Z_INS      =2
TOOL_ROT   =4
WRIST      =5
GRASP1     =6
GRASP2     =7
YAW        =8
GRASP      =9

JOINTS_ARRAY_INDICES = [SHOULDER, ELBOW, Z_INS, TOOL_ROT, WRIST, GRASP1, GRASP2, YAW, GRASP]
ROS_TO_L_OR = {0:2,
               1:3,
               2:4,
               4:5,
               5:6,
               6:8,
               7:10,
               8:7,
               9:9
               } 
ROS_TO_R_OR = {}
L_OR_TO_ROS = {2:0, # shoulder_L --> shoulder
               3:1, # elbow_L --> elbow
               4:2, # insertion_L--> z_ins
               5:4, # tool_roll_L--> tool_rot
               6:5, # wrist_joint_L--> wrist
               8:6, # grasper_joint_1_L --> grasp1
               10:7, #grasper_joint_2_L --> grasp2
               7:8, #grasper_yaw_L --> yaw
               9:9, #grasper1_tip_L --> grasp
               }
R_OR_TO_ROS = {}

prevFrame = None
currFrame = None
FramesLock = threading.Lock()

SHOULDER_MIN_LIMIT =(   0.0 * DEG2RAD)
SHOULDER_MAX_LIMIT =(  90.0 * DEG2RAD)
ELBOW_MIN_LIMIT =(  45.0 * DEG2RAD)
ELBOW_MAX_LIMIT =( 135.0 * DEG2RAD)

Z_INS_MIN_LIMIT =(-0.230)
Z_INS_MAX_LIMIT =( 0.010)

TOOL_ROLL_MIN_LIMIT =(-182.0 * DEG2RAD)
TOOL_ROLL_MAX_LIMIT =( 182.0 * DEG2RAD)
TOOL_WRIST_MIN_LIMIT =(-75.0 * DEG2RAD)
TOOL_WRIST_MAX_LIMIT =( 75.0 * DEG2RAD)

TOOL_GRASP_LIMIT = 89.0 * DEG2RAD
TOOL_GRASP1_MIN_LIMIT = (-TOOL_GRASP_LIMIT)
TOOL_GRASP1_MAX_LIMIT =   TOOL_GRASP_LIMIT
TOOL_GRASP2_MIN_LIMIT = (-TOOL_GRASP_LIMIT)
TOOL_GRASP2_MAX_LIMIT =   TOOL_GRASP_LIMIT

#========================= LEAP MOTION LISTENER CLASS ======================================================#
class Listener(Leap.Listener):
    def on_init(self, controller):
        print "Initialized"
        self.prevFrame = None
        self.currFrame = None
        self.rc = RavenController()

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
        grip = self.check_grip(frame)

        self.rc.updateActive(active)
        
        self.prevFrame = self.currFrame
        self.currFrame = frame 
        
        self.rc.run(self.prevFrame, self.currFrame, grip)
        #x = raw_input()

    def check_grip(self, frame):
        if not frame.hands.empty:
            # Get the first hand
            hand = frame.hands[0]
            # Check if the hand has any fingers
            fingers = hand.fingers
            if len(fingers) == 1:
                return True
            else:
                return False
        return False

    def check_active(self, frame):
        if not frame.hands.empty:
            # Get the first hand
            hand = frame.hands[0]
            # Check if the hand has any fingers
            fingers = hand.fingers
            if len(fingers) <= 2:
                return True
            else:
                return False
            """if len(fingers) <= 1:
                return False
            else:
                return True"""
        return False
    
#========================= RAVEN CONTROLLER CLASS ==========================================================#
class RavenController:
    def __init__(self, x_scale=X_SCALE, y_scale=Y_SCALE, z_scale=Z_SCALE, frame=None, relative_orientation=False, camera_frame=False):
        self.raven_pub = rospy.Publisher('raven_command', RavenCommand)
        self.x_scale = x_scale
        self.y_scale = y_scale
        self.z_scale = z_scale 
        self.active = False
        if MODE=="SIM":
            self.configureOREnv()  

    def updateActive(self, a):
        if self.active != a:
            if a==False:
                print "Controller inactive"
            else:
                print "Controller active"
        self.active = a 

    def run(self, p, c, grip):
        if self.active:
            if MODE == "REAL":
                raven_command = self.getRavenCommand()  # start continually publishing raven commands based on input from the leapmotion
                self.raven_pub.publish(raven_command)
            elif MODE == "SIM":
                joints, joints_array_indices = self.getORCommand(p, c, grip)
                self.publishORCommand(joints, joints_array_indices)  

    #========================== OPEN RAVE COMMANDS ================================#

    def configureOREnv(self):
        """ This function is called once at the begin to set up the openrave environment with the robot """
        env = rave.Environment()
        env.Load('myRaven.xml')
        self.robot = env.GetRobots()[0]
        self.manipulators = self.robot.GetManipulators()
        manip = self.manipulators[0]
        manipIndices = manip.GetArmIndices()
        allJoints = self.robot.GetJoints()  
        JointValues = [joint.GetValues()[0] for joint in allJoints]

        #These are the starting joints for the left arm; they are halfway between each limit
        leftJoints = [  (SHOULDER_MAX_LIMIT+SHOULDER_MIN_LIMIT)/2.0,
                        (ELBOW_MAX_LIMIT+ELBOW_MIN_LIMIT)/2.0,    
                        #(Z_INS_MAX_LIMIT+Z_INS_MIN_LIMIT)/2.0,
                        -0.1,
                        0,
                        (TOOL_ROLL_MAX_LIMIT+TOOL_ROLL_MIN_LIMIT)/2.0,
                        (TOOL_WRIST_MAX_LIMIT+TOOL_WRIST_MIN_LIMIT)/2.0,
                        (TOOL_GRASP1_MAX_LIMIT+TOOL_GRASP1_MIN_LIMIT)/2.0,
                        (TOOL_GRASP2_MAX_LIMIT+TOOL_GRASP2_MIN_LIMIT)/2.0,
                        0,
                        0
                        ]
        self.prevLeftJoints = leftJoints
        self.prevRightJoints = None 
        self.prevPose = fwdArmKin(0, leftJoints)[0] #FIXME: look at this
        env.SetViewer('qtcoin')

    def publishORCommand(self, joints, joints_array_indices):
        """ Publish a command to the robot; sets joints directly"""
        if joints != None and joints_array_indices != None:
            self.robot.SetJointValues(joints, joints_array_indices)

    def getORCommand(self, p, c, grip):
        """ Calculates joints based off of input from the leap motion """
        joints, joints_array_indices = self.calculateNewPose(p,c,grip)
        return joints, joints_array_indices


    #====================== OPEN RAVE HELPER FUNCTIONS =============================#
    def calculateNewPose(self, prev_frame, curr_frame, grip):
        prevPose = self.prevPose
        prev_x, prev_y, prev_z = prevPose.translation.x, prevPose.translation.y, prevPose.translation.z
        prevOrientation = prevPose.orientation
        if type(prev_frame) != type(None) and type(curr_frame) != type(None):
            translation, rotation = calculateTransform(prev_frame, curr_frame, 0)
            if type(translation) != type(None) and type(rotation) != type(None):
                dx, dy, dz = (translation[0]*self.x_scale, translation[1]*self.y_scale, translation[2]*self.z_scale)         
                x_basis, y_basis, z_basis = (rotation.x_basis.to_float_array(), rotation.y_basis.to_float_array(), rotation.z_basis.to_float_array())
                delta_rotation = np.matrix([x_basis, y_basis, z_basis])
                currOrientation = prevOrientation*delta_rotation
                print currOrientation
            else:
                dx, dy, dz = (0,0,0)
                currOrientation = prevOrientation
        else:
            dx, dy, dz = (0,0,0)
            currOrientation = prevOrientation
        curr_x, curr_y, curr_z = (prev_x+dx, prev_y+dy, prev_z+dz)
        try:
            prevjoints = invArmKin(0, self.prevPose, 0, False)
            currPose = tfx.pose(prevPose).copy()
            currPose.translation=(curr_x,curr_y,curr_z)
            currPose.orientation = currOrientation
            invkin_pass, joints, array_indices = self.calculateJoints(currPose, grip)
            if invkin_pass:
                self.prevPose = currPose
            return joints, array_indices
        except ValueError as v:
            return None, None
        


    def calculateJoints(self, pose, grip):
        result_joints_dict = invArmKin(0,pose,0)
        if result_joints_dict != None:
            joints = []
            joints_array_indices = []
            self.prevLeftJoints = []
            self.prevLeftJointsArrayIndices = []
            for key in sorted(result_joints_dict.keys()):
                joints.append(result_joints_dict[key])
                self.prevLeftJoints.append(result_joints_dict[key])
                joints_array_indices.append(ROS_TO_L_OR[key])
                self.prevLeftJointsArrayIndices.append(ROS_TO_L_OR[key])
            #print "INVERSE KINEMATICS DIDN'T FAIL"
            return True, joints, joints_array_indices 
        else:
            #print "INVERSE KINEMATICS FAILED"
            return False, self.prevLeftJoints, self.prevLeftJointsArrayIndices
        

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
    currHand = prevHand = None
    #print "PREV=================="
    #print str(prev.hands.empty)
    #print "CURR=================="
    #print str(curr.hands)
    if not prev.hands.empty: #FIXME WILL NEED TO GENERALIZE FOR TWO HANDS
        prevHand = prev.hands[index]
    if not curr.hands.empty: 
        currHand = curr.hands[index]
    """prev_palm_pos = prev.palm_position
    curr_palm_pos = curr.palm_position
    prev_palm_ori = (prevHand.normal.roll, prevHand.direction.pitch, prevHand.direction.yaw)
    curr_palm_ori = (currHand.normal.roll, currHand.direction.pitch, currHand.direction.yaw)"""
    if type(currHand) != type(None):
        translation = currHand.translation(prev) #this is a Leap.Vector!!
        rot_matrix  = currHand.rotation_matrix(prev) # this is a Leap.Matrix!!
        return translation, rot_matrix    
    else:
        #print "CURRHAND IS NONE"
        return None, None

#================= MAIN ================#
def main():
    # Create a sample listener and controller
    print "1"
    listener = Listener()
    controller = Leap.Controller()
    print "2"
    # Have the sample listener receive events from the controller
    controller.add_listener(listener)
    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    sys.stdin.readline()
    # Remove the sample listener when done
    controller.remove_listener(listener)

if __name__ == "__main__":
    main()
