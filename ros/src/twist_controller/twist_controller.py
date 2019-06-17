from yaw_controller import YawController
from pid import PID
import numpy as np
import math


GAS_DENSITY = 2.858
mph = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius

        self.throttle_controller = PID(1.0, 0.0, 0.0, -5.0, GAS_DENSITY)
        self.yaw_controller   = YawController(wheel_base, steer_ratio, mph, max_lat_accel, max_steer_angle)
        pass

    def control(self, enabled, current_lin_v, current_ang_v, target_lin_v, target_ang_v):
        if not enabled:
            self.throttle_controller.reset()
        lin_v_err = target_lin_v - current_lin_v

        acc = self.throttle_controller.step(lin_v_err, 1.0/50.0)
        throttle = 0.0
        brake    = 0.0
        if acc < 0:
            brake = -(acc * self.vehicle_mass * self.wheel_radius);
        else:
            throttle = acc / GAS_DENSITY;
        steer = self.yaw_controller.get_steering(target_lin_v, target_ang_v, current_lin_v)
        return throttle, brake, steer
