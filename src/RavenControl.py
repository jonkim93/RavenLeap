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
DEBUG = True

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
        print "here!!!!"

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
        self.prevFrame = self.currFrame
        self.currFrame = frame 
        self.rc.run(self.prevFrame, self.currFrame)

    
#========================= RAVEN CONTROLLER CLASS ==========================================================#
class RavenController:
    def __init__(self, scale=0.1, frame=None, relative_orientation=False, camera_frame=False):
        self.raven_pub = rospy.Publisher('raven_command', RavenCommand)
        self.scale = scale
        self.scale_increment = scale/20
        if MODE=="SIM":
            self.configureOREnv()  

    def run(self, p, c):
        if MODE == "REAL":
            raven_command = self.getRavenCommand()  # start continually publishing raven commands based on input from the leapmotion
            self.raven_pub.publish(raven_command)
        elif MODE == "SIM":
            joints = self.getORCommand(p, c)
            self.publishORCommand(joints)  

    #========================== OPEN RAVE COMMANDS ================================#

    def configureOREnv(self):
        env = rave.Environment()
        env.Load('myRaven.xml')
        self.robot = env.GetRobots()[0]
        self.manipulators = self.robot.GetManipulators()
        manip = self.manipulators[0]
        manipIndices = manip.GetArmIndices()

        allJoints = self.robot.GetJoints()  
        JointValues = [joint.GetValues()[0] for joint in allJoints]

        """leftJoints = [allJoints[2].GetValues()[0], #shoulder_L
                      allJoints[3].GetValues()[0], #elbow_L
                      allJoints[4].GetValues()[0], #insertion_L
                      allJoints[1].GetValues()[0], #filler--NOT NECESSARY AND SHOULD BE REMOVED
                      allJoints[5].GetValues()[0], #tool_roll_L
                      allJoints[6].GetValues()[0], #wrist_joint_L
                      allJoints[8].GetValues()[0], #grapser_joint_1_L
                      allJoints[10].GetValues()[0],#grasper_joint_2_L
                      allJoints[7].GetValues()[0], #grasper_yaw_L
                      allJoints[9].GetValues()[0]  #grasper1_tip_L #FIXME: WILL NEED TO CORREC THIS
                      ] """
        leftJoints = [  (SHOULDER_MAX_LIMIT+SHOULDER_MIN_LIMIT)/2.0,
                        (ELBOW_MAX_LIMIT+ELBOW_MIN_LIMIT)/2.0,    
                        (Z_INS_MAX_LIMIT+Z_INS_MIN_LIMIT)/2.0,
                        0,
                        (TOOL_ROLL_MAX_LIMIT+TOOL_ROLL_MIN_LIMIT)/2.0,
                        (TOOL_WRIST_MAX_LIMIT+TOOL_WRIST_MIN_LIMIT)/2.0,
                        (TOOL_GRASP1_MAX_LIMIT+TOOL_GRASP1_MIN_LIMIT)/2.0,
                        (TOOL_GRASP2_MAX_LIMIT+TOOL_GRASP2_MIN_LIMIT)/2.0,
                        0,
                        0
                        ]
        for joint in leftJoints:
            #print str(joint.GetValues())
            print str(joint)
        #leftJointValues = [value.GetValues()[0] for value in leftJoints]
        print "JOINTVALUES: "+str(leftJoints)
        self.prevPose = fwdArmKin(0, leftJoints)[0] #FIXME: look at this
        print "PREV POSEEEEEEEEE"
        print self.prevPose
        env.SetViewer('qtcoin')
        #x = input()
        

    def publishORCommand(self, joints):
        #set joint values
        print "JOINTS=============================="
        print joints
        if joints != None:
            joints_array_indices = [2,3,4,5,6,8,10,7,9]
            #self.robot.SetJointValues(joints.items(), JOINTS_ARRAY_INDICES)
            self.robot.SetJointValues(joints, joints_array_indices)
            x = input()
        else:
            print "5. NO JOINTS RECEIVED"
        x = input()
        IPython.embed()

    def getORCommand(self, p, c):
        print "1. FRAMES============================="
        print p
        print c
        prevPose = self.prevPose
        prev_x, prev_y, prev_z = prevPose.translation.x, prevPose.translation.y, prevPose.translation.z
        if type(p) != type(None) and type(c) != type(None):
            translation, rotation = calculateTransform(p, c, 0)
            #BASED OFF THIS TRANSLATION, WE NEED TO DIRECTLY CHANGE THE POSE SOMEHOW. HOWWW???
            if type(translation) != type(None) and type(rotation) != type(None):
                dx, dy, dz = (translation[0]*self.scale, 
                            translation[1]*self.scale, 
                            translation[2]*self.scale)
            else:
                dx, dy, dz = (0,0,0)
            print "2. Command Translation: "+str(dx)+", "+str(dy)+", "+str(dz)
        else:
            dx, dy, dz = (0,0,0)
        curr_x, curr_y, curr_z = (prev_x+dx, prev_y+dy, prev_z+dz)
        print "3. PREVIOUS JOINTS =========================================="
        print self.prevPose
        prevjoints = invArmKin(0, self.prevPose, 0, False)
        self.currPose = prevPose
        self.currPose.translation=(curr_x,curr_y,curr_z)
        self.prevPose = self.currPose
        #print "3. checking equality of current and prev poses"
        #print self.currPose==self.prevPose
        
        print prevjoints
        print "4. CURRENT POSE============================================"
        result_joints = invArmKin(0,self.currPose,0)
        #x = input()
        return result_joints
    

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
        print "CURRHAND IS NONE"
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
