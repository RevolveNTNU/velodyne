#!/usr/bin/env python

import requests
import rospy

# rosparams
rpm = rospy.get_param("/velodyne_nodelet_manager_driver/rpm")
ip = rospy.get_param("/velodyne_nodelet_manager_driver/device_ip")

# URLs
VELODYNE_URL = 'http://' + ip
FULL_URL = VELODYNE_URL + "/cgi/setting"


def post(payload):
	"""
	Post function that posts a payload to the given url.
	:param payload: Content of the post request
	"""
	r = requests.post(FULL_URL, data=payload)


def set_laser(laser):
	"""
	Set whether the laser is on or off
	:param laser:	0: Off
					1: On
	"""
	laser_set = {0: 'Off', 1: 'On'}
	if laser == 0 or laser == 1:
		payload = {'laser': laser_set[laser]}
		post(payload)


def set_rpm(rotation, is_hz=True):
	"""
	Set the rotation rate of the lidar. Can be set to both Hz and RPM. Defaulted to RPM.
	:param rpm:		RPM MIN:	500
					RPM MAX: 	1200

					Hz MIN: 	5
					Hz MAX:		20

	:param is_rpm:	Decides whether the rpm is set in Hz or RPM
	"""
	if is_hz:
		rotation = rotation * 60

	if 500 <= rotation <= 1200:
		payload = {'rpm': rotation}
		post(payload)


def set_fov(start, end):
	"""
	THIS DOES NOT WORK ATM

	:param start: 	Start of the FOV
	:param end: 	End of the FOV
	"""
	if 0 <= start < end <= 359:
		fov = {"start": start, "end": end}
		payload = {'fov': fov}
		post(payload)


def set_return_type(type):
	"""
	Sets the return type
	:param type: 	0: Strongest
					1: Last
					2: Dual
	"""
	type_set = {0: 'Strongest', 1: 'Last', 2: 'Dual'}
	if type == 0 or type == 1 or type == 2:
		payload = {'returns': type_set[type]}
		post(payload)


rospy.init_node('velodyne_utils', anonymous=False)
set_rpm(rpm, False)
