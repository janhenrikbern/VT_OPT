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
        self.max_delta = radians(45.0)

        # Info metrics:
        self.travel_dist = 0.0
        self.travel_time = 0.0

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

    # def step(self):
    #     self.v = self.a * self.dt
    #     self.dx = self.v * cos(self.delta)
    #     self.dy = self.v * sin(self.delta)

    #     self.location[0] += self.dx
    #     self.location[1] += self.dy

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
        if d_theta > self.max_delta: # max steering angle
            print(d_theta)
            print("can't reach target coordinate with current steering constraints")
            return None

        # steering negatively affects the speed. Need to slow down to turn.
        steering_correction = 1.0 - (d_theta / self.max_delta)
        dx, dy = self.get_distance_components(target_coor)
        dist = hypotenuse(dx, dy)
        v_gain = self.v + (dist / max(self.v, self.max_a)) * self.max_a
        v_adj = steering_correction * (self.v + min(v_gain, self.max_v)) / 2.0
        
        print(f"speed: {v_adj}")
        return dist / v_adj

    def get_location_and_heading(self):
        magnitude = hypotenuse(self.dx, self.dy)
        return self.location[0], self.location[1], self.dx / magnitude, self.dy / magnitude

    def plot_state(self):
        x, y, dx, dy = self.get_location_and_heading()
        plt.arrow(x, y, 5*dx, 5*dy, width=1.0)
    
    def __str__(self):
        out = "PointCar - Location (x,y): \n"
        for k, v in self.__dict__.items():
            out += f"{k}: {v}\n"
        return out

