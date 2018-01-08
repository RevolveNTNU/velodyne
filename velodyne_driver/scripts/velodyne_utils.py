#!/usr/bin/env python

import requests
import rospy
from std_msgs.msg import Int32


# rosparams
rpm_param = rospy.get_param("/velodyne_nodelet_manager_driver/rpm")
ip_param = rospy.get_param("/velodyne_nodelet_manager_driver/device_ip")

# URLs
VELODYNE_URL = 'http://' + ip_param
FULL_URL = VELODYNE_URL + "/cgi/setting"


current_rpm = rpm_param
rate = 10

def callback(rpm):
	global rpm_param
	rpm_param = rpm
	string = "rpm setpoint recieved from topic /rpm: "+str(rpm)
	rospy.loginfo(string)
	rpm = rpm.data
	set_rpm(rpm)

def update_rpm_param():
	global rpm_param
	rpm_param = rospy.get_param("/velodyne_nodelet_manager_driver/rpm")

def post(payload):
	"""
	Post function that posts a payload to the given url.
	:param payload: Content of the post request
	"""
	r = requests.post(FULL_URL, data=payload)
#	print (r.status_code)


def set_laser(laser):
	"""
	Set whether the laser is on or off
	:param laser: 0: Off, 1: On
	"""
	laser_set = {0: 'Off', 1: 'On'}
	if laser == 0 or laser == 1:
		payload = {'laser': laser_set[laser]}
		post(payload)


def set_rpm(rotation, is_hz=False):
	global current_rpm
	"""
	Set the rotation rate of the lidar. Can be set to both Hz and RPM. Defaulted to RPM.
	:param rpm: RPM MIN 500, RPM MAX: 1200
	"""
	if is_hz:
		rotation = rotation * 60
	if not 500 <= rotation <= 1200:
		return

	payload = {'rpm': rotation}
	post(payload)
	rospy.set_param('/velodyne_nodelet_manager_driver/rpm', str(rpm_param))
	current_rpm = rpm_param
	rpm_str = "RPM set to: "+ str(rotation)
	rospy.loginfo(rpm_str)


def set_fov(start, end):
	"""
	THIS DOES NOT WORK ATM
	:param start: Start of the FOV
	:param end: End of the FOV
	"""
	if 0 <= start < end <= 359:
		fov = {"start": start, "end": end}
		payload = {'fov': fov}
		post(payload)


def set_return_type(type):
	"""
	Sets the return type
	:param type: 0: Strongest, 1: Last, 2: Dual
	"""
	type_set = {0: 'Strongest', 1: 'Last', 2: 'Dual'}
	if type == 0 or type == 1 or type == 2:
		payload = {'returns': type_set[type]}
		post(payload)


set_rpm(rpm_param, False)



def main():
	global current_rpm
	global rpm_param
	global rate

	rospy.init_node('velodyne_utils', anonymous=False)
	rpm_sub = rospy.Subscriber("/rpm", Int32, callback)

	rate = rospy.Rate(rate)

	while not rospy.is_shutdown():
		update_rpm_param()
		if current_rpm != rpm_param:
			string = "Current RPM: "+str(current_rpm)+ "\tRPM_param: "+str(rpm_param)
			rospy.loginfo(string)
			set_rpm(current_rpm)
		rate.sleep()


if __name__ == "__main__": main()