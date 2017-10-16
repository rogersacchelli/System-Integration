from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# PID CONSTANTS

THROTTLE_KP = 0.4
THROTTLE_KI = 0.0001
THROTTLE_KD = 0.5

BREAKING_KP = 0.4
BREAKING_KI = 0.0001
BREAKING_KD = 0.5

PID_RATE = 1./20.

# LOW PASS FILTER VARIABLES 
LPF_TAU = 0.96
LPF_TS = 1

# Controller Variables
MIN_SPEED = 1

DEBUG_YAW = False

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement

        self.lpf_steer = LowPassFilter(LPF_TAU, LPF_TS)
        # Low Pass Filter will be applied to angular final angle
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED,\
        max_lat_accel, max_steer_angle)

        self.pid_throttle = PID(THROTTLE_KP, THROTTLE_KI, THROTTLE_KD,
                            mn=0, mx=1)

        self.pid_brake = PID(BREAKING_KP, BREAKING_KI, BREAKING_KD,
                            mn=0, mx=1)

        if(self.pid_brake is not None and self.pid_throttle is not None):
            rospy.loginfo("PID Initialized")

    def control(self, velocity_error, steer_list, is_enabled = False):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle = 0
        brake = 0
        steer = 0

        linear_velocity, angular_velocity, current_velocity = steer_list
        if DEBUG_YAW:
            rospy.loginfo("steer: %s|%s|%s" %(linear_velocity, angular_velocity,
                                            current_velocity))

        angular_velocity = self.lpf_steer.filt(angular_velocity)
        steer = self.yaw_controller.get_steering(linear_velocity,
                                            angular_velocity,
                                            current_velocity)

        throttle = self.pid_throttle.step(velocity_error, PID_RATE)
        brake = self.pid_brake.step(-velocity_error, PID_RATE)

        return throttle, brake, steer

    def reset_pid(self):
        self.pid_throttle.reset()
        self.pid_brake.reset()
