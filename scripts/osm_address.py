#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import geodesy.utm

import xml.etree.ElementTree as ET
tree = ET.parse('/home/mustafasezer/catkin_ws/src/cloud_visualizer/data/combined.osm')
root = tree.getroot()
nodes = root.findall('node')

pub = rospy.Publisher('/osm_map_coordinates', Pose2D, queue_size=10)

def callback(data):
    address = data.data.split()
    street = address[0]
    house_number = address[1]
    rospy.loginfo("I heard %s %s", street, house_number)
    latitude = -500
    longitude = -500
    node_found = False
    for node in nodes:
	number_found = False
	if(node_found):
	    break
	children = list(node)
	if(children):
           for child in list(node):
        	#if("addr:" in child.get('k')):
       	        #    print(child.get('v'))
		#if(child.get('k') == "addr:housenumber"):
		#    if(child.get('v') == "66"):
	   	#	print("FOUND")
		if(number_found):
		    if(child.get('k') == "addr:street" and street in child.get('v')):
			latitude = node.get('lat')
			longitude = node.get('lon')
			node_found = True
			break
            	if(child.get('k') == "addr:housenumber" and number_found==False):
		    if(child.get('v') != house_number):
			break
		    else:
			number_found = True

    """N5
    pose = Pose2D()
    pose.x = -248.29788208
    pose.y = 419.022888184
    pub.publish(pose)
    """

    if(latitude != -500 and longitude != -500):
	coord = geodesy.utm.fromLatLong(float(latitude), float(longitude)).toPoint()
	coord.x -= 691324
	coord.y -= 5335829
	print(coord.x, coord.y)
	pose = Pose2D()
	pose.x = coord.x
	pose.y = coord.y

	pose.x = -248.29788208
	pose.y = 419.022888184
	pub.publish(pose)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/osm_address", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
