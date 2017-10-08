
import math
import time
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_ctrl, pid_lin_vel, filt_ang_vel):
        self.yaw_ctrl = yaw_ctrl
        self.pid_lin_vel = pid_lin_vel
        self.filt_ang_vel = filt_ang_vel
        self.t0 = time.time()
        self.dt = 0.0
        self.vehicle_mass = None
        self.brake_deadband = None
        self.wheel_radius = None
        self.feed_forward_brake_control = True

    def set_vehicle_parameters(self, vehicle_mass, vehicle_mass_offset, brake_deadband, wheel_radius):
        self.vehicle_mass = vehicle_mass + vehicle_mass_offset
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius

    def control(self, current_velocity, linear_velocity, angular_velocity):
        # Return throttle, brake, steer

        # Check if all required parameters are set
        if not ( self.vehicle_mass and self.brake_deadband and self.wheel_radius):
            rospy.logerror('vehicle parameters not set')

        #Apply the filter to the angular velocity
        angular_velocity = self.filt_ang_vel.filt(angular_velocity)

        #Compute the steering angle
        steer = self.yaw_ctrl.get_steering(
                linear_velocity,
                angular_velocity,
                current_velocity)

        #Compute the throttle command
        cmd = 0
        vel_error = (linear_velocity - current_velocity)
        if self.dt:
            cmd = self.pid_lin_vel.step(vel_error, self.dt)

        self.dt = time.time() - self.t0
        self.t0 += self.dt

        #Apply the brakes if necessary
        if cmd > 0:
            throttle = cmd
            brake = 0.0
        else:
            # The correct brake torque can be computed using the desired acceleration,
            # weight of the vehicle and wheel radius with
            # Torque = Length x Force = Length x Mass x Accelecation
            throttle = 0.0

            if self.feed_forward_brake_control:
                desired_neg_acceleration = vel_error / self.dt
                brake = - self.wheel_radius * self.vehicle_mass * desired_neg_acceleration
                self.pid_lin_vel.reset() # reset controller to prevent windup
            else:
                # Alternatively use negative speed control
                brake = - self.wheel_radius * self.vehicle_mass * (cmd - self.brake_deadband)
            
            # If braking effort is small then don't use wheel brakes
            if math.fabs(cmd) < self.brake_deadband:
                # Car is currently still in the braking deadband, meaning that it is
                # not required to brake (the torque by the engine is enough for braking)
                brake = 0.0
            else:
                # The car needs to brake using wheel brakes
                pass
        
        # debug
        rospy.logdebug('cmd = %.2f, T = %.2f, B = %.2f, S = %.2f (BAND: %.2f)', cmd, throttle, brake, steer, self.brake_deadband)

        # The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.
        return throttle, brake, steer

    def reset(self):
    	self.pid_lin_vel.reset()
