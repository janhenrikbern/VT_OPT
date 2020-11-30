import matplotlib.pyplot as plt
import numpy as np
from math import (
    radians, sin, 
    cos, 
    sqrt,
    acos,
    asin,
    radians
)
from .utils import (
    hypotenuse
)

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
        self.dx = -0.00000001
        self.dy = -0.00000001

        self.location = list(starting_coordinates)
        self.delta = 0.0 # steering angle
        self.v = 0.0

        self.dt = 0.1 #sec
        self.max_v = 28.0 # m/s
        self.max_a = 7.0 # m/s^2
        self.max_turning_rate = radians(15.0) # rad / m based on a turning radius of 12 m

        # Info metrics:
        self.travel_dist = 0.0
        self.travel_time = 0.0

    def copy(self):
        vehicle = PointCar(self.location)
        for k, v in self.__dict__.items():
            vehicle.__dict__[k] = v
        return vehicle

    def update(self, node):
        # self.v = node.vel
        coor = (node.x, node.y)
        dist = hypotenuse(*self.get_distance_components(coor))
        self.travel_dist += dist
        time = self.get_time(coor)
        if time is not None:
            self.travel_time += time
            self.v = dist / time
        self.delta = self.heading(coor)
        self.location = coor

    def can_reach_location(self, coor):
        d1 = self.heading(coor)
        d0 = self.delta
        d_theta = abs(d1 - d0)
        dist = hypotenuse(*self.get_distance_components(coor))
        if d_theta / dist > self.max_turning_rate: # max steering angle
            # print(d_theta)
            # print("can't reach target coordinate with current steering constraints")
            return False
        return True

    def get_distance_components(self, coor):
        x1, y1 = coor
        x0, y0 = self.location
        dx = x1 - x0
        dy = y1 - y0
        return dx, dy

    def heading(self, coor):
        dx, dy  = self.get_distance_components(coor)
        return acos(dx / hypotenuse(dx, dy))

    def get_time(self, target_coor):
        d1 = self.heading(target_coor)
        d0 = self.delta
        d_theta = abs(d1 - d0)
        dx, dy = self.get_distance_components(target_coor)
        dist = hypotenuse(dx, dy)
        # steering negatively affects the speed. Need to slow down to turn.
        steering_correction = 1.0 - (d_theta / (self.max_turning_rate * dist))
        v_gain = self.v + (dist / max(self.v, self.max_a)) * self.max_a * steering_correction
        v_adj = (steering_correction * self.v + min(v_gain, steering_correction * self.max_v)) / 2.0
        
        # print(f"acceleration: {(v_adj - self.v) / (dist / v_adj)}")
        return dist / v_adj

    def get_location_and_heading(self):
        return self.location[0], self.location[1], cos(self.delta), sin(self.delta)

    def plot_state(self):
        x, y, dx, dy = self.get_location_and_heading()
        plt.arrow(x, y, 5*dx, 5*dy, width=1.0)
    
    def __str__(self):
        out = "PointCar: \n"
        for k, v in self.__dict__.items():
            out += f"{k}: {v}\n"
        return out

