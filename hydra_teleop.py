#!/usr/bin/env python
import roslib; roslib.load_manifest("raven_2_teleop")
import rospy
import math
from numpy import *
from numpy.linalg import *
import tf
import tf.transformations as tft
from sixense.msg import Calib, CalibPaddle
#from sixense.hydra import Hydra
from raven_2_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from raven_2_trajectory.srv import RecordTrajectory, RecordTrajectoryResponse
from optparse import OptionParser

SIDE_ACTIVE = [True,True]
SIDE_NAMES = ['L','R']
SIDE_NAMES_FRIENDLY = ['left','right']

BASE_FRAME = '/0_link'
END_EFFECTOR_FRAME_PREFIX = '/tool_'
END_EFFECTOR_FRAME_SUFFIX = ['L','R']

#CalibPaddle.START	= 0
#CalibPaddle.BUMPER   = 5
#CalibPaddle.JOYSTICK = 6

GRIP_CLOSE_BUTTON = [1,2]
GRIP_OPEN_BUTTON = [3,4]

START_RECORD_BUTTON = [2,1]
STOP_RECORD_BUTTON = [4,3]

SCALE_ADJUST_BUTTON = CalibPaddle.START
SCALE_UP_BUTTON = [3,4]
SCALE_DOWN_BUTTON = [1,2]
SCALE_RESET_BUTTON = CalibPaddle.JOYSTICK

DEFAULT_SCALE = .1

class SideChecker:

	def __init__(self,topic='raven_state'):
		self._msg = None
		self._sub = rospy.Subscriber(topic,RavenState,self._cb)

	def _cb(self,msg):
		if self._msg is None:
			self._msg = msg
			self._sub.unregister()

	def ready(self):
		return self._msg is not None

	def get_sides(self):
		if self._msg is None:
			return None
		return [arm.name for arm in self._msg.arms]

	def wait(self):
		rate = rospy.Rate(10)
		while not self.ready() and not rospy.is_shutdown():
			rate.sleep()


# left is paddle 0, right is paddle 1

class HydraTeleop:
	xoffset = 0
	yoffset = 0
	zoffset = 0
	last_msg = None
	start_recording_last = False
	stop_recording_last = False
	joystick_active = [True,True]
	recorder = None

	SIDE_ACTIVE = None

	def __init__(self,listener,record=False, scale=None, grasp_while_inactive=False, force_pedal_down=False, relative_orientation=False, frame=None, camera_frame=False):
		self.scale = scale
		self.scale_increment = scale / 20
		self.scale_adjusting_mode = False

		self.force_pedal_down = force_pedal_down

		self.grasp_while_inactive = grasp_while_inactive

		self.relative_orientation = relative_orientation

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

		self.pub = rospy.Publisher('raven_command', RavenCommand)

		# set up tf listener
		self.listener = listener

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

		print "subscribing to hydra"
		self.hydra_sub = rospy.Subscriber("hydra_calib", Calib, self.callback)

#		self.hydra = Hydra(callback=self.callback_new)
#		
#		self.hydra.add_named_button("activate",Hydra.BUMPER)
#		self.hydra.add_named_button("check side",Hydra.JOYSTICK_CLICK)
#		
#		self.hydra.add_named_button("grip close",2,mirror_to=Hydra.LEFT)
#		self.hydra.add_named_button("grip open",4,mirror_to=Hydra.LEFT)
#		
#		self.hydra.add_named_button("insertion",Hydra.X)
#		
#		self.hydra.add_named_button("scale adjust", Hydra.START)
#		self.hydra.add_named_button("scale reset",Hydra.JOYSTICK_CLICK)
#		self.hydra.add_named_button("scale up",4,mirror_to=Hydra.LEFT)
#		self.hydra.add_named_button("scale up",2,mirror_to=Hydra.LEFT)

		if record:
			self.recorder = rospy.ServiceProxy("record_trajectory",RecordTrajectory)

#			self.hydra.add_named_button("start recording",1,mirror_to=Hydra.LEFT)
#			self.hydra.add_named_button("stop recording",2,mirror_to=Hydra.LEFT)

		print "ready for action"

#	def callback_new(self,state):
#		#scale adjustment
#		if self.scale_adjusting_mode:
#			curr_scale = self.scale
#			for side in Hydra.SIDES:
#				paddle = msg.paddles[i]
#				if paddle.buttons[SCALE_ADJUST_BUTTON] and not self.last_msg.paddles[i].buttons[SCALE_ADJUST_BUTTON]:
#					self.scale_adjusting_mode = False
#					print "Done adjusting scale!"
#					self.last_msg = msg
#					return
#				
#				if paddle.buttons[SCALE_RESET_BUTTON] and not self.last_msg.paddles[i].buttons[SCALE_RESET_BUTTON]:
#					self.scale = DEFAULT_SCALE
#				elif paddle.buttons[SCALE_UP_BUTTON[i]] and not self.last_msg.paddles[i].buttons[SCALE_UP_BUTTON[i]]:
#					self.scale += self.scale_increment
#				elif paddle.buttons[SCALE_DOWN_BUTTON[i]] and not self.last_msg.paddles[i].buttons[SCALE_DOWN_BUTTON[i]]:
#					self.scale -= self.scale_increment
#				else:
#					time_diff = (msg.header.stamp - self.last_msg.header.stamp).to_sec()
#					time_per_increment = 0.4
#					#print time_diff / time_per_increment, time_diff
#					increment = self.scale_increment * time_diff / time_per_increment
#					self.scale += increment * paddle.joy[1]
#			if self.scale < 0:
#				self.scale = 0;
#			if curr_scale != self.scale:
#				print "Scale is now %1.4f" % self.scale 
#			self.last_msg = msg
#			return

	def callback(self,msg):
		if self.last_msg is None: 
			self.last_msg = msg
			return

		#scale adjustment
		if self.scale_adjusting_mode:
			curr_scale = self.scale
			for i in xrange(2):
				paddle = msg.paddles[i]
				if paddle.buttons[SCALE_ADJUST_BUTTON] and not self.last_msg.paddles[i].buttons[SCALE_ADJUST_BUTTON]:
					self.scale_adjusting_mode = False
					print "Done adjusting scale!"
					self.last_msg = msg
					return

				if paddle.buttons[SCALE_RESET_BUTTON] and not self.last_msg.paddles[i].buttons[SCALE_RESET_BUTTON]:
					self.scale = DEFAULT_SCALE
				elif paddle.buttons[SCALE_UP_BUTTON[i]] and not self.last_msg.paddles[i].buttons[SCALE_UP_BUTTON[i]]:
					self.scale += self.scale_increment
				elif paddle.buttons[SCALE_DOWN_BUTTON[i]] and not self.last_msg.paddles[i].buttons[SCALE_DOWN_BUTTON[i]]:
					self.scale -= self.scale_increment
				else:
					time_diff = (msg.header.stamp - self.last_msg.header.stamp).to_sec()
					time_per_increment = 0.4
					#print time_diff / time_per_increment, time_diff
					increment = self.scale_increment * time_diff / time_per_increment
					self.scale += increment * paddle.joy[1]
			if self.scale < 0:
				self.scale = 0;
			if curr_scale != self.scale:
				print "Scale is now %1.4f" % self.scale 
			self.last_msg = msg
			return

		#print "got msg"

		raven_command = RavenCommand()
		raven_command.header.stamp = rospy.Time.now()
		raven_command.header.frame_id = BASE_FRAME

		raven_command.controller = Constants.CONTROLLER_CARTESIAN_SPACE

		active = [False,False]
		newly_active = [False,False]
		for i in xrange(2):
			if SIDE_ACTIVE[i] and msg.paddles[i].buttons[CalibPaddle.BUMPER]:
				active[i] = True
				newly_active[i] = not self.last_msg.paddles[i].buttons[CalibPaddle.BUMPER]
		grip = [0,0]

		for i in xrange(2):

			if not SIDE_ACTIVE[i]:
				continue

			arm_cmd = ArmCommand()
			tool_cmd = ToolCommand()
			tool_cmd.pose_option = ToolCommand.POSE_OFF
			paddle = msg.paddles[i]

			if (not active[i]) and paddle.buttons[SCALE_ADJUST_BUTTON] and not self.last_msg.paddles[i].buttons[SCALE_ADJUST_BUTTON]:
				print "Adjusting scale!  Current value: %1.7f" % self.scale
				self.scale_adjusting_mode = True
				self.last_msg = msg
				return

			if paddle.buttons[CalibPaddle.JOYSTICK] and not self.last_msg.paddles[i].buttons[CalibPaddle.JOYSTICK]:
				print "That's the %s controller!" % SIDE_NAMES_FRIENDLY[i]

			if newly_active[i]:
				print "%s tool active" % SIDE_NAMES_FRIENDLY[i]
				# calculate current position of robot
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

				#print "engaging %s arm"%self.arms[i].lr
			elif not paddle.buttons[CalibPaddle.BUMPER] and self.last_msg.paddles[i].buttons[CalibPaddle.BUMPER]:
				print "%s tool inactive" % SIDE_NAMES_FRIENDLY[i]


			if active[i] and not newly_active[i]:

				dx = (paddle.transform.translation.x  - self.last_msg.paddles[i].transform.translation.x) * self.scale
				dy = (paddle.transform.translation.y  - self.last_msg.paddles[i].transform.translation.y) * self.scale
				dz = (paddle.transform.translation.z  - self.last_msg.paddles[i].transform.translation.z) * self.scale

				x = paddle.transform.translation.x * self.scale + self.xoffset
				y = paddle.transform.translation.y * self.scale + self.yoffset
				z = paddle.transform.translation.z * self.scale + self.zoffset

				xx = paddle.transform.rotation.x
				yy = paddle.transform.rotation.y
				zz = paddle.transform.rotation.z
				ww = paddle.transform.rotation.w

				p = mat(array([dx,dy,dz,1])).transpose()

				T1 = mat(array([[0,1,0,0],  [-1,0,0,0],  [0,0, 1,0], [0,0,0,1]]))
				#T2 = mat(array([[1,0,0,0],  [0,-1,0,0],  [0,0,-1,0], [0,0,0,1]]))

				qmat = T1 * mat(tft.quaternion_matrix(array([xx,yy,zz,ww])))

				if self.relative_orientation:
					last_xx = self.last_msg.paddles[i].transform.rotation.x
					last_yy = self.last_msg.paddles[i].transform.rotation.y
					last_zz = self.last_msg.paddles[i].transform.rotation.z
					last_ww = self.last_msg.paddles[i].transform.rotation.w

					last_qmat = T1 * mat(tft.quaternion_matrix(array([last_xx,last_yy,last_zz,last_ww])))

					qmat = inv(last_qmat) * qmat

				p_t = array(T1 * p)[0:3].flatten().tolist()
				q_t = array(tft.quaternion_from_matrix(qmat)).flatten().tolist()

				if self.relative_orientation:
					tool_cmd.pose_option = ToolCommand.POSE_RELATIVE
				else:
					tool_cmd.pose_option = ToolCommand.POSE_POS_REL_ORI_ABS
				tool_cmd.pose = Pose(Point(*p_t),Quaternion(*q_t))

				if paddle.buttons[GRIP_OPEN_BUTTON[i]]: grip[i] = 1
				if paddle.buttons[GRIP_CLOSE_BUTTON[i]]: grip[i] = -1

				if self.joystick_active[i]:
					if paddle.joy[1] and not self.last_msg.paddles[i].joy[1]:
						print "Insertion active"
					j_cmd = JointCommand()
					j_cmd.command_type = JointCommand.COMMAND_TYPE_VELOCITY
					j_cmd.value = paddle.joy[1]
					arm_cmd.joint_types.append(Constants.JOINT_TYPE_INSERTION)
					arm_cmd.joint_commands.append(j_cmd)

					if paddle.joy[0] and not self.last_msg.paddles[i].joy[0]:
						pass
						#print "Rotation active"
					#raven_command.joint_velocities[i].rotation = paddle.joy[0]

				#if paddle.buttons[CalibPaddle.START] or paddle.joy[0] or paddle.joy[1]:
				if paddle.trigger or paddle.joy[0] or paddle.joy[1]:
					tool_cmd.pose.position = Point(0,0,0)

			arm_cmd.active = active[i] and SIDE_ACTIVE[i]

			tool_cmd.grasp_option = ToolCommand.GRASP_INCREMENT_SIGN
			if arm_cmd.active:
				tool_cmd.grasp = grip[i]
			elif SIDE_ACTIVE[i] and self.grasp_while_inactive:
				if paddle.buttons[GRIP_OPEN_BUTTON[i]]: grip[i] = 1
				if paddle.buttons[GRIP_CLOSE_BUTTON[i]]: grip[i] = -1
				if grip[i]:
					arm_cmd.active = True
					tool_cmd.grasp = grip[i]
					tool_cmd.pose_option = ToolCommand.POSE_OFF

			arm_cmd.tool_command = tool_cmd
			raven_command.arm_names.append(SIDE_NAMES[i])
			raven_command.arms.append(arm_cmd)

			if self.recorder:
				try:
					if paddle.buttons[START_RECORD_BUTTON[i]] and not self.start_recording_last:
						resp = self.recorder(True,False,"")
						if resp.result == RecordTrajectoryResponse.SUCCESS:
							print "started recording"
						elif resp.result == RecordTrajectoryResponse.ERROR:
							print "recording did not start: %d" % resp.result
					elif paddle.buttons[STOP_RECORD_BUTTON[i]] and self.stop_recording_last:
						resp = self.recorder(False,False,"")
						if resp.result == RecordTrajectoryResponse.SUCCESS:
							print "stopped recording"
						elif resp.result == RecordTrajectoryResponse.ERROR:
							print "error while stopping recording: %d" % resp.result
				except rospy.ServiceException, e:
					print "Recording call failed: %s" % e




		#publish RavenCommand
		if self.force_pedal_down:
			raven_command.pedal_down = True
		else:
			raven_command.pedal_down = any([arm.active for arm in raven_command.arms])

		self.pub.publish(raven_command)

		self.last_msg = msg
		self.start_recording_last = msg.paddles[0].buttons[START_RECORD_BUTTON[0]] or msg.paddles[1].buttons[START_RECORD_BUTTON[1]]
		self.stop_recording_last = msg.paddles[0].buttons[STOP_RECORD_BUTTON[0]] or msg.paddles[1].buttons[STOP_RECORD_BUTTON[1]]

if __name__ == "__main__":
	rospy.init_node("teleop",anonymous=True)

	parser = OptionParser()

	parser.add_option('--disable-left',action='store_true',default=False)
	parser.add_option('--disable-right',action='store_true',default=False)

	parser.add_option('-g','--grasp-while-inactive',action='store_true',default=False)

	parser.add_option('-s','--scale',default=DEFAULT_SCALE)

	parser.add_option('-r','--record',action='store_true',default=False)

	parser.add_option('--force-pedal-down',action='store_true',default=False)

	parser.add_option('-o','--relative-orientation',action='store_true',default=False)

	(options,args) = parser.parse_args()

	side_checker = SideChecker()
	side_checker.wait()

	sides = side_checker.get_sides()
	for idx, side_name in enumerate(SIDE_NAMES):
		if side_name not in sides:
			print 'Auto-disabling side %s' % SIDE_NAMES_FRIENDLY[idx]
			SIDE_ACTIVE[idx] = False

	if options.disable_left:
		SIDE_ACTIVE[0] = False
	if options.disable_right:
		SIDE_ACTIVE[1] = False

	listener = tf.TransformListener()

	HT = HydraTeleop(listener,record=options.record,scale=options.scale,grasp_while_inactive=options.grasp_while_inactive,
					force_pedal_down=options.force_pedal_down,relative_orientation=options.relative_orientation)
	rospy.spin()

	rospy.loginfo('shutting down')
