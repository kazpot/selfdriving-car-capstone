#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float64
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
import math

from twist_controller import Controller
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

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

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5) # 'brake' is a torque
        accel_limit = rospy.get_param('~accel_limit', 1.) #  'accel' is a percentage
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        min_speed = rospy.get_param('~min_speed', 0.1)
        Kp = rospy.get_param('~pid_kp', 1.1)
        Ki = rospy.get_param('~pid_ki', 0.010)
        Kd = rospy.get_param('~pid_kd', 0.005)
        pid_cmd_range = rospy.get_param('~pid_cmd_range', 4)
        filter_tau = rospy.get_param('~filter_tau', 0.0)

        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)

        self.vx_pub = rospy.Publisher('/debug_vx', Float64, queue_size=1)
        self.sz_pub = rospy.Publisher('/debug_sz', Float64, queue_size=1)
        self.vxd_pub = rospy.Publisher('/debug_vxd', Float64, queue_size=1)
        self.szd_pub = rospy.Publisher('/debug_szd', Float64, queue_size=1)

        # def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio,
                                            min_speed, max_lat_accel,
                                            max_steer_angle)

        # def __init__(self, tau, ts):
        self.filter = LowPassFilter(filter_tau, 0.8) # hm why 0.8?

        # Pid controller for the target velocity (decel_limit is already negative)
        self.pid_vel = PID(Kp, Ki, Kd, decel_limit, accel_limit)

        # write controller
        self.controller = Controller(self.yaw_controller, self.pid_vel, self.filter)

        # set controller parameters
        vehicle_mass_offset = 25.0 + 70.0 + 30.0 # additional weight (gas, passenger, load)
        self.controller.set_vehicle_parameters(vehicle_mass, vehicle_mass_offset, brake_deadband, wheel_radius)

        self.current_velocity = None
        self.target_velocity = None
        self.dbw_enabled = True # maybe change to False by default
        self.pose = None

        rospy.Subscriber("/current_velocity", TwistStamped, self.velocity_cb)
        rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cb)
        rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        self.loop()

    def velocity_cb(self, msg):
        self.current_velocity = msg.twist
        self.vx_pub.publish(self.current_velocity.linear.x)
        self.sz_pub.publish(self.current_velocity.angular.z)

    def twist_cb(self, msg):
        self.target_velocity = msg.twist
        self.vxd_pub.publish(self.target_velocity.linear.x)
        self.szd_pub.publish(self.target_velocity.angular.z)

    def dbw_cb(self, msg):
        self.dbw_enabled = msg.data
        rospy.loginfo('received dbw_enabled: %s', str(msg.data))

    def pose_cb(self, msg):
        self.pose = msg.pose

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

            if not self.current_velocity:
                rospy.logwarn('no current velocity has been set')
                rate.sleep()
                continue

            if not self.target_velocity:
                rospy.logwarn('no target velocity has been set')
                rate.sleep()
                continue

            if not self.pose:
                rospy.logwarn('no pose has been set')
                rate.sleep()
                continue

            if not self.dbw_enabled:
                rospy.logdebug('no driving by wire')
                #Reset the PID controller
                self.controller.reset()
                rate.sleep()
                continue

            throttle, brake, steer = self.controller.control(
               current_velocity=self.current_velocity.linear.x,
               linear_velocity=self.target_velocity.linear.x,
               angular_velocity=self.target_velocity.angular.z
            )

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
