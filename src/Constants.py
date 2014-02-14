#!/usr/bin/env python

#========================= ARGUMENTS ============================#
MODE = "SIM" # or "REAL"
ARM = "ONE_ARM" # or "TWO_ARM"

EXP = False 

DEBUG = True

DEG2RAD = 0.0174532925

INTERPOLATION_WEIGHTS = (0.5,0.5,0.5)
JOINT_WEIGHTS = (1,1,1,1,1,1,1,1,1)

X_SCALE=0.00017
Y_SCALE=0.00017
Z_SCALE=0.00021

DX_UPPER_BOUND = 0.003
DY_UPPER_BOUND = 0.003
DZ_UPPER_BOUND = 0.003

MAX_TWO_FINGER_DIST = 90.0
MIN_TWO_FINGER_DIST = 40.0

MAX_HAND_SPHERE_RADIUS = 130.0
MIN_HAND_SPHERE_RADIUS = 70.0

#========================= CONSTANTS ============================#
MODEL_NAME = "myRaven.xml" 

SHOULDER   =0
ELBOW      =1
Z_INS      =2
TOOL_ROT   =4
WRIST      =5
GRASP1     =6
GRASP2     =7
YAW        =8
GRASP      =9

ACTIVE_THRESHOLD = 5

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