import numpy as np
import matplotlib.pyplot as plt
from matplotlib import image
from math import sin, cos

class PointCar:
    def __init__(self, *starting_coordinates):
        """
        Simulates point model with steering inputs of acceleration (a) and steering_angle.
        Dynamics based on point model as described in:
        https://www2.eecs.berkeley.edu/Pubs/TechRpts/2017/EECS-2017-102.pdf

        car = PointCar(0, 0)

        Args:
            *starting_coordinates: starting location of point car

        Returns:
            None
        """
        # control inputs
        self.a = 0.0
        self.steering_angle = 0.0

        # constant car attributes
        self.weight = 1.0


        self.location = list(starting_coordinates)

        self.dt = 0.1 #sec

    def step(self):
        v = self.a * self.dt
        dx = self.v * cos(self.steering_angle)
        dy = self.v * sin(self.steering_angle)

        self.location[0] += dx
        self.location[1] += dy

    def get_location_and_heading(self):
        return self.location[0], self.location[1], self.steering_angle

