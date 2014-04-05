#!/usr/bin/env python

from Constants import *

#========== HELPER FUNCTIONS ==========#


def calculateDeltaCommand(translation, x_scale, y_scale, z_scale):
    return (translation[0]*x_scale, translation[2]*z_scale, translation[1]*y_scale)

def scaleDeltaCommand(command, weights=INTERPOLATION_WEIGHTS):
    results = []
    for i in range(0,len(command)):
        results.append(command[i]*weights[i])
    return results 

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
        palm_normal = currHand.palm_normal
        palm_direction = currHand.direction
        return translation, rot_matrix, palm_normal, palm_direction    
    else:
        #print "CURRHAND IS NONE"
        return None, None