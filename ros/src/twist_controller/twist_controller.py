import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius,
                 wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, .1, max_lat_accel, max_steer_angle)

        kp = .3
        ki = .1
        kd = 0.
        mn = 0. # Minimum throttle value
        mx = 10. # Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = .5
        ts = .02
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()
        
        self.pid_throttle = PID(0.35,0.0,0.0,0.0,1.0)
        self.pid_brake = PID(0.5,0.0,0.0,0.0,1.0)
        #Low pass filter for steering cmd
        self.lowpass_steer = LowPassFilter(0.2,1.0)


    def control(self, dbw_enabled, current_vel, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        steer = self.yaw_controller.get_steering(target_speed, target_angular_speed, current_speed)
        steer = self.lowpass_steer.filt(steer)

        cte = target_speed - current_speed
        throttle = self.pid_throttle.step(cte, time_elapsed)
        brake = self.pid_brake.step(-cte, time_elapsed)

        # return 0.3, 0., steer
        return throttle, brake, steer
