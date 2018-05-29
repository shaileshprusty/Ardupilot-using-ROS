#!/usr/bin/env python
import rospy 
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
import csv
from math import sin, cos, sqrt, atan2, radians, degrees, pi
import utm
from mavros_msgs.msg import AttitudeTarget
import geopy.distance
from numpy.linalg import norm 
import geopy 
from geopy.distance import VincentyDistance
import numpy as np
# import numpy as np 
import math
import matplotlib.pyplot as plt

## Global Variables
inc = 1     # incrementer for the VTP
kp = 0.01     # P gain
ki = 0.0001           # I gain
curHead = 0  # heading 
iError = 0      # intergral error


# Returns the current heading
def hdgCallback(data):
	global curHead
	curHead = data.data
	# print "Global Heading", curHead



# This function returns the lat,long of the bot, calls the carrot function and publishes the velocity commands
def posCallback(data):	

	wayList = readWaypoints()
	r = rospy.Rate(1)
	p = [data.latitude, data.longitude] # current locaiton of the bot



 	print "Current Location of Bot", p

	# Getting the current heading
	rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, hdgCallback)
	global curHead		

	# To check distance from the final position
	finalWP = [float(i) for i in wayList[number]]

	finalWP = utm.from_latlon(finalWP[0], finalWP[1] )
	p = utm.from_latlon(p[0], p[1] )

	# thetau = atan2(p[1] - finalWP[1], p[0] - finalWP[0])
	thetau = atan2(finalWP[1] - p[1], finalWP[0] - p[0])
	print "Desired Heading:", degrees(thetau)
	# global des
	# des = degrees(thetau)



	# 	pubVelocity(control)

	

def velCallback(data):
	print(data.twist.linear.x)

def location():
	# Subscribing to the (lat, long) of the robot	
	rospy.Subscriber("/mavros/global_position/global", NavSatFix, posCallback)



# Reads the waypoints of the path to be followed and returns it
def readWaypoints():
	wlist = []
	with open('mission.txt', 'rb') as f:
		reader = csv.reader(f)
		for row in reader:
			# print row[0], row[1]
			wlist.append(row)

		return wlist


# Returns the euclidean distance
def euclideanDis(v, w):
	return sqrt((w[1] - v[1])**2 + (w[0] - v[0])**2)


# Returns the euclidean distance between lat/long coordinates
def euclideanDisGPS(v, w):
	v = utm.from_latlon(v[0], v[1] )
	w = utm.from_latlon(w[0], w[1] )
	return sqrt((w[1] - v[1])**2 + (w[0] - v[0])**2)



###############################



def pot_fields(finalLat, finalLong):

#	print "Hellloooooji"

	# Obstacle Coordinates 28.544379, 77.271688
#	xObst = 28.544379
#	yObst = 77.271688
	
	# Goal Coordinates
	xGoal = finalLat
	yGoal = finalLong

	# Goal Parameters
	gRadius = 1
	sGoal = 10
	alpha = 0.8

	# Obstacle Parameters
#	oRadius = 3
#	sObst = 10	
#	beta = 0.6	
	

	# Current lat/long and goal lat/long
	xCurr = currLat
	yCurr = currLong
	


	distanceGoal = euclideanDisGPS((xCurr,yCurr),(xGoal,yGoal))  # distance to goal
#	distanceObs = euclideanDisGPS((xCurr,yCurr),(xObst,yObst))   # distance to obstacle


	### Calculating thetas #### 
	utmCur = utm.from_latlon(xCurr, yCurr)
#	utmObs = utm.from_latlon(xObst, yObst)
	utmGoal = utm.from_latlon(xGoal, yGoal)

	thetaG = atan2(utmGoal[1] - utmCur[1], utmGoal[0] - utmCur[0])
#	thetaO = atan2(utmObs[1] - utmCur[1], utmObs[0] - utmCur[0])
	###########################


	##### Goal Delx and Dely ######
	if (distanceGoal < gRadius):
		delxG = 0
		delyG = 0
	elif (distanceGoal> gRadius and distanceGoal<(sGoal+gRadius)):
		delxG = alpha*(distanceGoal - gRadius)*cos(thetaG)
		delyG = alpha*(distanceGoal - gRadius)*sin(thetaG)
	else:
		delxG = alpha*sGoal*cos(thetaG)
		delyG = alpha*sGoal*sin(thetaG)
	######################


	##### Obstacle Delx and Dely ######
#	if (distanceObs < oRadius):
#		delxO = -100*np.sign(cos(thetaO))
#		delyO = -100*np.sign(sin(thetaO))
#	elif (distanceObs> oRadius and distanceGoal<(sObst+oRadius)):
#		delxO = -beta*(sObst + oRadius - distanceObs)*cos(thetaO)
#		delyO = -beta*(sObst + oRadius - distanceObs)*sin(thetaO)
#	else:
#		delxO = 0
#		delyO = 0
	######################

#	netdelx = delxG + delxO
#	netdely = delyG + delyO

        netdelx = delxG
        netdely = delyG

	vmag = sqrt(netdelx**2 + netdely**2)
	desTheta = degrees(atan2(netdely, netdelx))

	return vmag, desTheta


currAng=0

def heading(msg):
	global currAng
	x = 1
	currAng = msg.data


def calculateError(des, curr):
	#
	if(curr<90 and curr>0):
		theta_cnew = 90 - curr 
		error = des - theta_cnew
	else:
		theta_cnew = 90 - curr
		error = des - theta_cnew

	error = error%360
	if(error>180):
		error = error -360

	return error 


currLat = 0.0
currLong = 0.0
finalLat = 0.0
finalLong = 0.0


number =0

def getLoc(data):	

	p = [data.latitude, data.longitude] # current locaiton of the bot

	global currLat
	global currLong
	currLat = data.latitude
	currLong = data.longitude
	
des = 0
def algo():
	
	velocityPub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size = 10)
	headingSub = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, heading)
	rospy.Subscriber("/mavros/global_position/global", NavSatFix, getLoc)
	rate = rospy.Rate(1) #1 H
	wayList = readWaypoints()
	global finalLong
	global finalLat
	global number
	number =0

	r = rospy.Rate(1)
	location()
	# rospy.spin()

	while not rospy.is_shutdown():
		
		# print 'hii', float(finalLat)
		# print type(float(finalLong))
		finalLat = float(wayList[number][0])
		finalLong = float(wayList[number][1])
	

		pointA = (float(currLat), float(currLong))
		# pointD = (float(28.544689), float(77.273489))
		pointD = (float(finalLat), float(finalLong))

		# des = calculate_initial_compass_bearing(pointA,pointD)
		Wi1 = utm.from_latlon(currLat, currLat)
		Wi2 = utm.from_latlon(finalLat, finalLong)
		# des = atan2(Wi2[1] - Wi1[1], Wi2[0] - Wi1[0])

		vmag, des = pot_fields(finalLat, finalLong)
		print 'Des', des

		print 'heading angle', currAng

		

		error = calculateError(des, currAng)
		print 'error', error

		move = TwistStamped()

		check = euclideanDisGPS(pointD, pointA)

		
		
		print "distance from final waypoint:", check


		if(check < 2):  # if in the vicinity, then stop giving the control
			move.twist.linear.x = 0
			move.twist.linear.y = 0
			move.twist.angular.z = 0
			velocityPub.publish(move)

			

			if(number<(len(wayList)-1)):
				number +=1
				print 'len', len(wayList)
				print 'number', number

			
		else:
			move.twist.linear.x = 3
			move.twist.linear.y = 3
			move.twist.angular.z = error*kp
			velocityPub.publish(move)
		
		# x = input()		
		r.sleep()

	rospy.spin()



###################################################################################



def mainFunc():
	rospy.init_node('rover_WP_nav', anonymous=True)
	algo()


# Main function
if __name__ == '__main__':
	try:
		mainFunc()
	except rospy.ROSInterruptException:
		pass
