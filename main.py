import matplotlib.pyplot as plt
import numpy as np

from track import load_track, is_valid_location
from car import PointCar
import random
import pure_viterbi


def find_valid_trajectory(track):
	pass

def optimize_trajectory(track, trajectory):
	pass


if __name__ == "__main__":
	IMG_PATH = "./tracks/loop.png"
	track = load_track(IMG_PATH)

	# Set to a valid point in trajectory
	car = PointCar(50, 100)
	fig = plt.figure()
	
	plt.imshow(track)
	for i in range(300):
		print(f"Step {i+1}")
		car.update(5.0, 1.0 + (random.random() - 0.5))
		car.step()
		car.plot_state()
		plt.pause(0.01)
	plt.show()
	print(car)