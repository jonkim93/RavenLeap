#!/usr/bin/env python

#====== OPENRAVE =========#
import openravepy as rave

#====== ROS      =========#
#import tf
#import tf.transformations as tft
import tfx
#import roslib; roslib.load_manifest("raven_2_teleop")
#import rospy
#from raven_2_msgs.msg import *
#from geometry_msgs.msg import Pose, Point, Quaternion
#from raven_2_trajectory.srv import RecordTrajectory, RecordTrajectoryResponse

from General_Helper import *
#====== CUSTOM SCRIPTS ===#
from RavenKin import *
from Constants import *

import Leap

from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

#====== GENERAL ==========#
import sys
import getopt
import math
import numpy as np 
from numpy import *
from numpy.linalg import *
from optparse import OptionParser

import IPython
import getch

#====== CUSTOM SCRIPTS ===#
from RavenKin import *
from Constants import *
from RavenControllers import *

#========================= RAVEN CONTROLLER SUPERCLASS ========================================================================#

class RavenController(object):
    def __init__(self, x_scale=X_SCALE, y_scale=Y_SCALE, z_scale=Z_SCALE, frame=None, relative_orientation=False, camera_frame=False):
        self.x_scale = x_scale
        self.y_scale = y_scale
        self.z_scale = z_scale 
        self.active = False 

    
    # ----  THESE METHODS NEED TO BE OVERRIDDEN ---- #
    def run(self, p, c, grip, tipDistance):
        if self.active:
        	pass

    def calculateNewPose(self):
    	pass


    # ----  THESE METHODS ARE FINE AS IS ---------- #
    def updateActive(self, a):
        if self.active != a:
            if a==False:
                print "Controller inactive"
            else:
                print "Controller active"
        self.active = a 

    def calculateDeltaPose(self, prev_frame, curr_frame, grip, tipDistance):  #TODO: MEASURE CHANGE IN POSE -- ABSOLUTE? 
        #ROTATION = tfx.pose([0,0,0],[0,180,90]).matrix 
        prevPose = self.prevPose
        #prevPose = ROTATION*prevPose 
        prev_x, prev_y, prev_z = prevPose.translation.x, prevPose.translation.y, prevPose.translation.z
        prevOrientation = prevPose.orientation
        if type(prev_frame) != type(None) and type(curr_frame) != type(None): #if we actually have frames
            translation, rotation = calculateTransform(prev_frame, curr_frame, 0)
            if type(translation) != type(None) and type(rotation) != type(None): # if we have a valid translation/rotation
                dx, dy, dz = (translation[0]*self.x_scale, translation[2]*self.y_scale, translation[1]*self.z_scale)         
                #dx, dy, dz = scaleDeltaCommand(calculateDeltaCommand(translation, self.x_scale, self.y_scale, self.z_scale))  #FIXME: THIS MAY NOT WORK

                x_basis, y_basis, z_basis = (rotation.x_basis.to_float_array(), rotation.y_basis.to_float_array(), rotation.z_basis.to_float_array())
                delta_rotation = np.matrix([x_basis, y_basis, z_basis])
                currOrientation = tfx.tb_angles(-90,90,0)
                #print currOrientation

                if math.fabs(dx) > DX_UPPER_BOUND or math.fabs(dy) > DY_UPPER_BOUND or math.fabs(dz) > DZ_UPPER_BOUND:
                    (dx, dy, dz) = (0,0,0)
                    currOrientation = prevOrientation
            else:  # DOC: if we don't have a valid translation/rotation, don't move
                dx, dy, dz = (0,0,0)
                currOrientation = prevOrientation
        else: # if we don't have frames, also don't move
            dx, dy, dz = (0,0,0)
            currOrientation = prevOrientation
        #print "dx, dy, dz = "+str(dx)+", "+str(dy)+", "+str(dz)
        curr_x, curr_y, curr_z = (prev_x+dx, prev_y+dy, prev_z+dz) # ADD IN THE DELTA COMMAND
        return (dx, dy, dz), currOrientation 


#========================= OPENRAVE SIMULATION RAVEN CONTROLLER CLASS ==========================================================#
class OR_RavenController(RavenController):
    def __init__(self, grip, x_scale=X_SCALE, y_scale=Y_SCALE, z_scale=Z_SCALE, frame=None, relative_orientation=False, camera_frame=False):
        super(OR_RavenController, self).__init__(x_scale, y_scale, z_scale)
        self.configureOREnv()
        self.grip_type = grip   

    def updateActive(self, a):
        if self.active != a:
            if a==False:
                print "Controller inactive"
            else:
                print "Controller active"
        self.active = a 

    def run(self, p, c, grip, tipDistance):
        if self.active:
            joints, joints_array_indices = self.getORCommand(p, c, grip, tipDistance)
            self.publishORCommand(joints, joints_array_indices)  

    #========================== OPEN RAVE COMMANDS ================================#

    def configureOREnv(self):
        """ This function is called once at the begin to set up the openrave environment with the robot """
        self.env = rave.Environment()
        self.env.Load('myRaven.xml')
        self.robot = self.env.GetRobots()[0]
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
        self.env.SetViewer('qtcoin')

    def publishORCommand(self, joints, joints_array_indices):
        """ Publish a command to the robot; sets joints directly"""
        if joints != None and joints_array_indices != None:
            self.robot.SetJointValues(joints, joints_array_indices)

    def getORCommand(self, p, c, grip, tipDistance):
        """ Calculates joints based off of input from the leap motion """
        joints, joints_array_indices = self.calculateNewPose(p,c,grip, tipDistance)
        return joints, joints_array_indices


    #====================== OPEN RAVE HELPER FUNCTIONS =============================#
    def calculateNewPose(self, prev_frame, curr_frame, grip, tipDistance):  #TODO: would like to streamline this
        #ROTATION = tfx.pose([0,0,0],[0,180,90]).matrix 
        prevPose = self.prevPose
        #prevPose = ROTATION*prevPose 
        prev_x, prev_y, prev_z = prevPose.translation.x, prevPose.translation.y, prevPose.translation.z
        prevOrientation = prevPose.orientation
        
        (dx, dy, dz),currOrientation = super(OR_RavenController, self).calculateDeltaPose(prev_frame, curr_frame, grip, tipDistance) # check out this line

        curr_x, curr_y, curr_z = (prev_x+dx, prev_y+dy, prev_z+dz) # ADD IN THE DELTA COMMAND

        try:
            prevJoints = invArmKin(0, self.prevPose, 0, False)

            # set new pose values
            currPose = tfx.pose(prevPose).copy()
            currPose.translation=(curr_x,curr_y,curr_z)
            currPose.orientation = currOrientation

            # calculate new joints
            invkin_pass, nextJoints, array_indices = self.calculateJoints(currPose, grip, tipDistance)


            #==============  THIS STUFF IS EXPERIMENTAL ===========#
            # weight joints
            if EXP:
                newJoints = scaleAndAddDeltaJoints(prevJoints,nextJoints)
                print "HERE"
                try:
                    print "JOINT WEIGHTS: "+JOINT_WEIGHTS
                    newPose = fwdArmKin(0, newJoints)[0]
                    resultJoints = newJoints
                except Exception:
                    print "JOINT WEIGHTING FAILED"
                    newPose = currPose
                    resultJoints = nextJoints   #============== END EXPERIMENTAL STUFF ================#
            else:
                resultJoints = nextJoints
                newPose = currPose             
            if invkin_pass:
                self.prevPose = newPose
            return resultJoints, array_indices
        except ValueError as v:
            print "VALUE ERROR"
            return None, None
        


    def calculateJoints(self, pose, grip, tipDistance):
        result_joints_dict = invArmKin(0,pose,0)
        if result_joints_dict != None:
            prevLeftJointsCopy = self.prevLeftJoints
            if EXP:
                joints = {}
                self.prevLeftJoints = {}
            else:
                joints = []
                self.prevLeftJoints = []
            joints_array_indices = []
            self.prevLeftJointsArrayIndices = []
            #print "GRASP JOINTS???"


            if self.grip_type == "h":
                max_distance = MAX_HAND_SPHERE_RADIUS
                min_distance = MIN_HAND_SPHERE_RADIUS
            elif self.grip_type == "t":
                max_distance = MAX_TWO_FINGER_DIST
                min_distance = MIN_TWO_FINGER_DIST

            #    SET GRIP HERE
            if not grip and tipDistance>=40:
                result_joints_dict[6] = 0.78*((tipDistance-min_distance)/max_distance)  #INTERPOLATE TIP DISTANCE HERE
                result_joints_dict[7] = 0.78*((tipDistance-min_distance)/max_distance)
            elif not grip and tipDistance < 40:
                result_joints_dict[6] = prevLeftJointsCopy[5] 
                result_joints_dict[7] = prevLeftJointsCopy[6]
            else:
                result_joints_dict[6] = 0
                result_joints_dict[7] = 0

            for key in sorted(result_joints_dict.keys()):
                if EXP:
                    joints[ROS_TO_L_OR[key]] = result_joints_dict[key]
                    self.prevLeftJoints[ROS_TO_L_OR[key]] = result_joints_dict[key]
                else:
                    joints.append(result_joints_dict[key]) #  MAY NEED TO BRING THIS BACK
                    self.prevLeftJoints.append(result_joints_dict[key])
                joints_array_indices.append(ROS_TO_L_OR[key])
                self.prevLeftJointsArrayIndices.append(ROS_TO_L_OR[key])
            #print "INVERSE KINEMATICS DIDN'T FAIL"
            return True, joints, joints_array_indices 
        else:
            #print "INVERSE KINEMATICS FAILED"
            return False, self.prevLeftJoints, self.prevLeftJointsArrayIndices
        

#========================= ROS RAVEN CONTROLLER CLASS ==========================================================================#
class ROS_RavenController(RavenController):
    def __init__(self, grip, x_scale=X_SCALE, y_scale=Y_SCALE, z_scale=Z_SCALE, frame=None, relative_orientation=False, camera_frame=False):
        self.raven_pub = rospy.Publisher('raven_command', RavenCommand)
        self.x_scale = x_scale
        self.y_scale = y_scale
        self.z_scale = z_scale 
        self.active = False
        self.grip_type = grip 

    def run(self, p, c, grip, tipDistance):
        if self.active:
            raven_command = self.getRavenCommand(p,c,grip,tipDistance)  # start continually publishing raven commands based on input from the leapmotion
            self.raven_pub.publish(raven_command)

    #========================== ROBOT/ROS COMMANDS =================================#
    def getRavenCommand(self, p, c, grip, tipDistance):
        """
        Calculates a raven command based off of LeapMotion frames
        """
        raven_command = RavenCommand()
        raven_command.header.stamp = rospy.Time.now()
        raven_command.header.frame_id = BASE_FRAME
                
        raven_command.controller = Constants.CONTROLLER_CARTESIAN_SPACE
        leftArmCommand = self.getArmCommand(0, p, c, grip, tipDistance)
        rightArmCommand = self.getArmCommand(1, p, c, grip, tipDistance)
        raven_command.arm_names = ['left', 'right']
        raven_command.arms = [leftArmCommand, rightArmCommand]

        #FIXME: need to account for pedal down and up in leapmotion commands
        raven_command.pedal_down = True
        return raven_command

    def getArmCommand(self, arm, p, c, grip, tipDistance):
        """
        Given an arm index, calculates and returns the proper arm command based off of the LeapMotion frames 
        """
        
        (dx, dy, dz), newOrientation = super(RavenController, self).calculateDeltaPose(p, c, grip, tipDistance)

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
            #print "dx: "+dx+"  dy: "+dy+"  dz: "+dz+"\n"

            #FIXME: this stuff is probably wrong
            
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

#=========== OR HELPER FUNCTIONS   =======================
def calculateDeltaJoints(prevJoints,nextJoints):
    deltaJoints = []
    #print str(prevJoints.keys())
    #print str(nextJoints.keys())
    for key in sorted(nextJoints.keys()):
        deltaJoints.append(nextJoints[key]-prevJoints[L_OR_TO_ROS[key]])
    return deltaJoints 

def scaleAndAddDeltaJoints(prevJoints,nextJoints,weights=JOINT_WEIGHTS):
    deltaJoints = calculateDeltaJoints(prevJoints,nextJoints)
    scaledDeltaJoints = [delta*weight for delta,weight in deltaJoints,weights]
    newJoints = []
    for i in range(0,len(prevJoints)):
        newJoints.append(prevJoints[i]+scaledDeltaJoints[i])
    return newJoints

def plot_transform(env, T, s=0.1):
    """
    Plots transform T in openrave environment.
    S is the length of the axis markers.
    """
    h = []
    x = T[0:3,0]
    y = T[0:3,1]
    z = T[0:3,2]
    o = T[0:3,3]
    h.append(env.drawlinestrip(points=np.array([o, o+s*x]), linewidth=3.0, colors=np.array([(1,0,0),(1,0,0)])))
    h.append(env.drawlinestrip(points=np.array([o, o+s*y]), linewidth=3.0, colors=np.array(((0,1,0),(0,1,0)))))
    h.append(env.drawlinestrip(points=np.array([o, o+s*z]), linewidth=3.0, colors=np.array(((0,0,1),(0,0,1)))))
    return h













