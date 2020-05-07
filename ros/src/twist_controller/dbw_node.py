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

RATE_SAMPLING = 50 #50Hz

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        max_speed = rospy.get_param('waypoint_loader/velocity', 40) / 3.6 # from kph to mps
        min_speed = rospy.get_param('~min_speed', 0.5)
        steering_tau = rospy.get_param('~steering_tau', 0.0)
        throttle_kp = rospy.get_param('~throttle_k_p', 0.5)
        throttle_ki = rospy.get_param('~throttle_k_i', 0.00001)
        throttle_kd = rospy.get_param('~throttle_k_d', 0.0)

        throttle_gains = [throttle_kp, throttle_ki, throttle_kd]

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.dbw_enabled = False
        self.vel_curr = None
        self.twist_cmd = None

        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)
        self.controller = Controller(
                                        accel_limit = accel_limit,
                                        brake_deadband = brake_deadband,
                                        decel_limit = decel_limit,
                                        fuel_capacity = fuel_capacity,
                                        max_lat_accel = max_lat_accel,
                                        max_steer_angle = max_steer_angle,

                                        max_speed = max_speed,
                                        min_speed = min_speed,

                                        sample_rate = RATE_SAMPLING,
                                        steer_ratio = steer_ratio,
                                        steering_tau = steering_tau,
                                        throttle_gains = throttle_gains,
                                        vehicle_mass = vehicle_mass,
                                        wheel_base = wheel_base,
                                        wheel_radius = wheel_radius
                                    )

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        
        self.loop()


    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data
        #rospy.logwarn("dbw_enabled_cb: %s", self.dbw_enabled)
        if self.dbw_enabled == True:
            self.controller.reset()


    def curr_vel_cb(self,msg):
        self.vel_curr = msg.twist
        #rospy.logwarn("twist_velocity_cb: %s", self.vel_curr)

    
    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg.twist
        #rospy.logwarn("twist_cmd_cb: %s", self.twist_cmd)



    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            rate = rospy.Rate(RATE_SAMPLING)
            while not rospy.is_shutdown():
                if self.dbw_enabled == True:
                    throttle, brake, steer = self.controller.control(self.twist_cmd, self.vel_curr)
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
