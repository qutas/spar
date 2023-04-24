#!/usr/bin/env python3

import rospy

from breadcrumb.srv import RequestPath
from breadcrumb.srv import RequestPathRequest

if __name__ == '__main__':
	# Setup ROS
	rospy.init_node('path_planner', anonymous=True)

	# List of waypoints (XYZ-Yaw)
	waypoints = [[-1.0,-1.0,0.0,0.0],
				 [ 1.5,-1.0,0.0,0.0],
				 [ 1.5, 1.5,0.0,0.0]]

	# Wait to connect with Breadcrumb
	# Code will error if you try to connect to a service
	# that does not exist
	rospy.wait_for_service('/breadcrumb/request_path')
	srvc_bc = rospy.ServiceProxy('/breadcrumb/request_path', RequestPath)

	# Loop through the list of waypoints
	for i in range(len(waypoints)-1):
		# Set up a path request for breadcrumb
		req = RequestPathRequest()
		req.start.x = waypoints[i][0]
		req.start.y = waypoints[i][1]
		req.start.z = waypoints[i][2]
		req.end.x = waypoints[i+1][0]
		req.end.y = waypoints[i+1][1]
		req.end.z = waypoints[i+1][2]

		res = srvc_bc(req)

		# Breadcrumb will return a vector of poses if a solution was found
		# If no solution was found (i.e. no solution, or request bad
		# start/end), then breadcrumb returns and empty vector
		# XXX: You could also use res.path_sparse (see breadcrumb docs)
		if len(res.path.poses) > 0:
			# Print the path to the screen
			rospy.loginfo("Segment %i:", i+1)
			rospy.loginfo("[%0.2f;%0.2f;%0.2f] => [%0.2f;%0.2f;%0.2f]",
						  req.start.x,req.start.y,req.start.z,
						  req.end.x,req.end.y,req.end.z)

			# Loop through the solution returned from breadcrumb
			for i in range(len(res.path.poses) - 1):
				rospy.loginfo("    [%0.2f;%0.2f;%0.2f]",
							  res.path.poses[i].position.x,
							  res.path.poses[i].position.y,
							  res.path.poses[i].position.z)
		else:
			rospy.logerr("solution not found")