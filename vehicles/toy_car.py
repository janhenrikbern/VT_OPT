# import numpy as np
from math import (
    sin, 
    cos,
    atan,
    tanh,
    sqrt,
    radians
)


class ToyCar:
    def __init__(self, *starting_coordinates) -> None:
        """
        Simulates point model with steering inputs of acceleration (a) and steering_angle.
        Dynamics based on kinematic bicycle model as described in:
        https://www2.eecs.berkeley.edu/Pubs/TechRpts/2017/EECS-2017-102.pdf

        car = ToyCar(0, 0, 0.0)

        Args:
            *starting_coordinates: starting location of point car and heading angle

        Returns:
            None
        """
        # control inputs
        self.a = 0.0 # acceleration
        self.theta = 0.0 # steering angle

        # car properties
        self.car_length = 2.0
        self.max_a = 7.0 # max. acceleration (0-100 in ~4 sec)
        self.max_v = 28.0 # max. speed of ~100 km / h
        self.max_theta = radians(30.0) # max. turn in front wheels
        self.com_offset = 0.0 # center of mass offset of the car

        # car state variables
        self.v = 0.0 # velocity
        self.beta = 0.0 # heading angle
        self.location = list(starting_coordinates)

        # Global time
        # self.dt = 0.1 #sec

    def update(self, acceleration, steering_input):
        self.a = acceleration
        self.steering_angle = steering_input

    def get_time(self, target_coor):
        x1, y1 = target_coor
        x0, y0 = self.location
        dx = x1 - x0
        dy = y1 - y0
        b = self.beta
        d = self.theta
        

    def step(self):
        self.v = self.a * self.dt
        self.dx = self.v * cos(self.steering_angle)
        self.dy = self.v * sin(self.steering_angle)

        self.location[0] += self.dx
        self.location[1] += self.dy

    def get_location_and_heading(self):
        magnitude = sqrt(self.dx**2 + self.dy**2)
        return self.location[0], self.location[1], self.dx / magnitude, self.dy / magnitude

    def plot_state(self):
        x, y, dx, dy = self.get_location_and_heading()
        plt.arrow(x, y, 5*dx, 5*dy, width=1.0)
    
    def __str__(self):
        return f"PointCar - Location (x,y): {self.location}, \n acceleration: {self.a}, \n velocity: {self.v}"
