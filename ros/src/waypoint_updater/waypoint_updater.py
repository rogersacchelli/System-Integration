#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.base_waypoints = None
        self.target_velocity = None
        self.current_pose = None
        self.current_pose_position = None
        self.current_pose_orientation = None
        self.final_topic = None
        self.pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1, latch=True)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber("/traffic_waypoint", Waypoint, self.traffic_cb)
        #rospy.Subscriber("/obstacle_waypoint", Waypoint, self.obstacle_cb)


        # TODO: Add other member variables you need below
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg
        #rospy.loginfo(self.current_pose.pose)
        self.publish()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        rospy.loginfo("Receiving waypoints from file - %s" % type(waypoints))
        self.base_waypoints = waypoints.waypoints
        self.publish()

    def publish(self):

        def _get_distance_from_pose(pose, wpt):
            return math.sqrt((pose.x-wpt.x)**2 + (pose.y-wpt.y)**2 + \
            (pose.z-wpt.z)**2)

        #rate = rospy.Rate(10) # 10 Hz
        if(self.current_pose is not None and self.base_waypoints is not None):
            start_idx = 0
            ego_position = self.current_pose.pose.position
            wpt_of_interest = self.base_waypoints[start_idx].pose.pose.position
            eval_dist = _get_distance_from_pose(ego_position, wpt_of_interest)
            for i in range(1, len(self.base_waypoints)):
                wpt_of_interest = self.base_waypoints[i].pose.pose.position
                wpt_dist = _get_distance_from_pose(ego_position, wpt_of_interest)
                #rospy.loginfo("Calculate dist for entry %d, %f" %(i, wpt_dist))
                if(wpt_dist <= eval_dist):
                    #rospy.loginfo("New shortest point found %d" %(i))
                    eval_dist = wpt_dist
                    start_idx = i

                    self.final_topic = self.base_waypoints[start_idx:start_idx+LOOKAHEAD_WPS]

            # Publish limited set of waypoints
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time(0)
            lane.waypoints = self.final_topic
            rospy.loginfo("x:%s|y:%s index-> %d "%(ego_position.x,
                                                    ego_position.y,
                                                    start_idx))
            self.pub.publish(lane)
            #rate.sleep()

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        # Calculate distance between two points. Usefull for distance between
        # non straight lines.

        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
