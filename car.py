import matplotlib.pyplot as plt
from math import (
    sin, 
    cos, 
    sqrt,
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
        self.steering_angle = 0.0 

        self.location = list(starting_coordinates)

        self.dt = 0.1 #sec

    def update(self, acceleration, steering_input):
        self.a = acceleration
        self.steering_angle = steering_input

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
        return f"PointCar - Location (x,y): {self.location}, acceleration: {self.a}"

