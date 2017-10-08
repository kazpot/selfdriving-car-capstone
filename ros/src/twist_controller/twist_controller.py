import rospy
import pid
import lowpass
from yaw_controller import YawController
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, vehicle_mass, brake_deadband, wheel_radius):
        # TODO: Implement
        self.dt = 0.0
        self.timestamp = time.time()
        self.steer_filter = lowpass.LowPassFilter(tau=0.0, ts=1.0)
        self.vel_pid = pid.PID(kp=1.3, ki=0.05, kd=1.4, mn=-5, mx=1)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        self.vehicle_mass = vehicle_mass
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius

    def control(self, current_velocity, linear_velocity, angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steeri

        angular_velocity = self.steer_filter.filt(angular_velocity)
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity) 
        vel_err = linear_velocity - current_velocity
        cmd = 0
        brake = 0.0
        if self.dt:
            cmd = self.vel_pid.step(vel_err, self.dt)
        else:
            self.dt = time.time() - self.timestamp
            self.timestamp += self.dt

        if cmd > 0:
            throttle = cmd
            brake = 0.0
        else:
            throttle = 0.0
            desired_accel = vel_err / self.dt
            brake = -self.wheel_radius * self.vehicle_mass * (cmd - self.brake_deadband)


        rospy.loginfo("steering angle: %s", steer)
        return throttle,brake,steer
