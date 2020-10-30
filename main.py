import numpy as np
import matplotlib.pyplot as plt
from matplotlib import image
from numpy.linalg.linalg import norm

IMG_PATH = "./tracks/track.png"

def load_track(img_path):
	img = image.imread(img_path)
	img_array = np.asarray(img)
	track_matrix = transform_to_track(img_array)
	return track_matrix

def transform_to_track(img):
	matrix = np.sum(img, axis=2)
	normalized = matrix / np.linalg.norm(matrix)
	return (normalized / np.max(normalized)) < 0.5

track = load_track(IMG_PATH)
plt.imshow(track)
plt.show()


def find_valid_trajectory(track):
	pass


def optimize_trajectory(track, trajectory):
	pass

