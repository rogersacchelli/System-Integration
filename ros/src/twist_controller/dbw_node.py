#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

DEBUG_CONTROLLER = True
DEBUG_DBW = False
DEBUG_CONTROL = True


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -3.)
        accel_limit = rospy.get_param('~accel_limit', 3.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.controller = Controller(wheel_base, steer_ratio, max_lat_accel,
                                        max_steer_angle)


        # Controll variables
        self.throttle = 0   #[0 - 1]
        self.steer = 0   #[-1 - 1]
        self.brake_deadband = brake_deadband

        # Status variables
        self.sub_dbw_enabled = False
        self.sub_cur_linear_x = None
        self.sub_cur_vel = None
        self.angular_z = None
        self.linear_x  = None   # Also desired_velocity
        self.desired_velocity = None
        self.sub_twist_cmd = None
        self.sub_cur_vel_linear_x = None


        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)

        self.loop()


    def current_velocity_cb(self, msg):
        self.sub_cur_vel = msg
        self.sub_cur_linear_x = self.sub_cur_vel.twist.linear.x

    def dbw_cb(self, msg):
        self.sub_dbw_enabled = msg.data
        rospy.loginfo("DBW: %s" %self.sub_dbw_enabled)
        self.controller.reset_pid()


    def twist_cb(self, msg):
        if msg is not None:
            self.sub_twist_cmd = msg
            self.linear_x = self.sub_twist_cmd.twist.linear.x
            self.angular_z = self.sub_twist_cmd.twist.angular.z


    def loop(self):

        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
                    #rospy.loginfo("Yaw Angle %s" %self.steer)
            # Controller
            rate = rospy.Rate(50) # 50Hz

            if(self.linear_x is not None and self.sub_cur_linear_x is not None):
                    if self.controller is not None:
                        steer_list = [self.linear_x,
                                    self.angular_z,
                                    self.sub_cur_linear_x]

                        velocity_cte = self.linear_x - self.sub_cur_linear_x

                        throttle, brake, steer = self.controller.control(
                                                        velocity_cte,
                                                        steer_list,
                                                        self.sub_dbw_enabled)
                        if DEBUG_CONTROLLER:
                            rospy.loginfo("controller: %s|%s|%s" %(throttle, brake, steer))

                        #if brake <= self.brake_deadband:
                        #    brake = 0

                        if self.sub_dbw_enabled:
                            if DEBUG_CONTROL:
                                rospy.loginfo("velocity:%s|desired_velocity:%s"
                                    %(self.sub_cur_linear_x, self.linear_x))

                            self.publish(throttle, brake, steer)
            rate.sleep()


    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
