
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
import openravepy as rave

from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

BASE_FRAME = '/0_link'
END_EFFECTOR_FRAME_PREFIX = '/tool_'
END_EFFECTOR_FRAME_SUFFIX = ['L','R']
SIDE_ACTIVE = [True,False]
SIDE_NAMES = ['L','R']
SIDE_NAMES_FRIENDLY = ['left','right']

prevFrame = None
currFrame = None
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
        FramesLock.acquire()
        if prevFrame == None:
            currFrame = frame
        else:
            prevFrame = currFrame
            currFrame = frame
        FramesLock.release()

    

class RavenController:
    def __init__(self, scale=0.1, frame=None, relative_orientation=False, camera_frame=False):
        self.raven_pub = rospy.Publisher('raven_command', RavenCommand)
        self.scale = scale
        self.scale_increment = scale/20
        env = rave.Environment()
        env.Load('myRaven.xml')
        self.robot = robot.GetRobots()[0]
        self.manipulators = robot.GeManipulators()
        manip = manipulators[0]
        manipIndices = manip.GetArmIndices()
        # set joint values
        # robot.SetJointsValues(jointPositionArray, jointIndicesArray)
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
            raven_command = self.getRavenCommand()  # start continually publishing raven commands based on input from the leapmotion
            self.raven_pub.publish(raven_command)

        # Remove the sample listener when done
        controller.remove_listener(listener)        

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
            tool_cmd.pose_option = ToolCommand.POSE_RELATIVE #FIXME: this sets the toolcommand to relative pose commands; user should be able to specify

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


def main():
    rc = RavenController()


if __name__ == "__main__":
    main()
