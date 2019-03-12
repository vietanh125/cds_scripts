import numpy as np
import tensorflow as tf
from tf.keras.layers import Dense
import time

class PID:
    def __init__(self, Kp = 0.0, Ki = 0.0, Kd = 0.0, setpoint = 0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0
        # self.max_integral = 0.0
        self.derivative = 0.0
        self.setpoint = setpoint
        self.last_time = time.time()

    def set_Kp(self, Kp):
        self.Kp = Kp

    def set_Ki(self, Ki):
        self.Ki = Ki

    def set_Kd(self, Kd):
        self.Kd = Kd

    def set_max_integral(self, max_integral):
        self.max_integral

    def change_setpoint(self, setpoint):
        self.setpoint = setpoint

    def update(self, value):
        current_time = time.time()
        dt = current_time - self.last_time
        error = self.setpoint - value
        self.integral += error * dt
        self.derivative = (error - self.previous_error)/dt
        output = self.Kp*error + self.Ki*self.integral + self.Kd*self.derivative
        self.previous_error = error
        self.last_time = current_time
        return output
