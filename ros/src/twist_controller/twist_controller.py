from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.yaw_control = YawController(#Steering control - low pass filter
                                            kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            kwargs['min_speed'],
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'],
                                        )
        self.lpf = LowPassFilter(kwargs['steering_tau'], 1.0/kwargs['sample_rate'])

        pid_gains = kwargs['throttle_gains']
        self.pid = PID(pid_gains[0], pid_gains[1], pid_gains[2], kwargs['max_speed'])

        total_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity']*GAS_DENSITY
        self.max_brake_torque = total_mass * kwargs['decel_limit'] * kwargs['wheel_radius']
        self.min_brake = -1.0 * kwargs['brake_deadband']
        #pass


    def control(self, value_target, value_curr):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        u = self.pid.step(value_target.linear.x, value_curr.linear.x, rospy.get_time())

        if u > 0:
            # Acceleration control
            throttle = max(0.0, min(1.0, u))
            brake = 0.0
        else:
            # Deceleration control
            throttle = 0.0
            brake = self.max_brake_torque * min(self.min_brake, u/self.pid.max_abs_u)
        
        #rospy.logwarn("[twist_controller.py - control - line47] self.max_brake_torque = %f", self.max_brake_torque)
        #rospy.logwarn("[twist_controller.py - control - line48] self.min_brake = %f", self.min_brake)
        #rospy.logwarn("[twist_controller.py - control - line49] self.pid.max_abs_u = %f", self.pid.max_abs_u)

        # Steering control
        #steering = self.lpf.filt(self.yaw_control.get_steering(value_target.linear.x, value_target.angular.z, value_curr.linear.x))
        steering = self.yaw_control.get_steering(value_target.linear.x, value_target.angular.z, value_curr.linear.x)
        steering = self.lpf.filt(steering)
                
        #return 1., 0., 0.
        return throttle, brake, steering

    def reset(self, *args, **kwargs):
        self.pid.reset()