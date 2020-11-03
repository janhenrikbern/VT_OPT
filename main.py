import matplotlib.pyplot as plt

from track import load_track
from car import PointCar
import random



def find_valid_trajectory(track):
	pass

def optimize_trajectory(track, trajectory):
	pass


if __name__ == "__main__":
	IMG_PATH = "./tracks/track.png"
	track = load_track(IMG_PATH)

	# Set to a valid point in trajectory
	car = PointCar(50, 100)
	fig = plt.figure()
	
	for i in range(500):
		print(f"Step {i+1}")
		car.update(5.0, 1.0 + (random.random() - 0.5))
		car.step()
		# print(car)

		if i % 100 == 0:
			plt.imshow(track)
			car.plot_state()
			plt.show()
