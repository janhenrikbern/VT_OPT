import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from track import load_track
from car import PointCar


def equal_separate_contours(contours):
    contours = np.squeeze(contours[1])
    return contours[::5]


# we can put this in a class
def get_inner_contours(track):
    track = track.astype(np.uint8)
    track = np.expand_dims(track, axis=2)
    contours, hierarchy = cv2.findContours(track, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    contours = equal_separate_contours(contours)
    return contours


def find_closest_point(location, contour):
    x, y = location
    closest = [0, 0]
    closest_index = 0
    distance = 9999999
    for index, point in enumerate(contour):
        if math.pow((x-point[0]),2) + math.pow((y-point[1]),2) < distance:
            closest = point
            closest_index = index
            distance = math.pow((x-point[0]),2) + math.pow((y-point[1]),2)
    return closest, closest_index


# move a slight distance to find the positive direction
def find_direction(car, index, contours):
    x, y, dx, dy = car.get_location_and_heading()
    x_ = x + 5*dx
    y_ = y + 5*dy
    v = np.array([x_-x, y_-y])
    v1 = np.array([contours[index+3][0]-contours[index][0], contours[index+3][1]-contours[index][1]])
    v2 = np.array([contours[index-3][0]-contours[index][0], contours[index-3][1]-contours[index][1]])

    if np.dot(v, v1) > np.dot(v, v2):
        return 1
    else:
        return -1


def find_valid_trajectory(car, track):
    visible = 199
    width = 10

    contours = get_inner_contours(track)

    # horizontal to the left is the original angle
    (x, y), index = find_closest_point(car.location, contours)
    direction = find_direction(car, index, contours)
    
    if direction > 0:
        if index + visible*direction > len(contours):
            available_points = np.concatenate((contours[index:len(contours)-1],contours[0:index + visible * direction - len(contours)-1]), axis=0)
        else:
            available_points = contours[index:index + visible*direction]
    else:

        if index < -visible*direction:
            available_points = np.concatenate((contours[index + visible * direction:], contours[0:index]), axis=0)
        else:
            available_points = contours[index + visible*direction: index]

    states_list = []
    available_points = available_points[::-1]
    for index, ap in enumerate(available_points):
        states = []
        if not index == 0 and not index == visible-1:
            horiz_v = np.array([available_points[index-1][0]-available_points[index+1][0], available_points[index-1][1]-available_points[index+1][1]])
            vertical_v = np.array([horiz_v[1], -horiz_v[0]])
            if len(states_list) == 0:
                point_to_p = np.array([car.location[0] - available_points[index][0], car.location[1] - available_points[index][1]])
            else:
                point_to_p = np.array([states_list[-1][-1][0] - available_points[index][0], states_list[-1][-1][1] - available_points[index][1]])
            if np.dot(vertical_v, point_to_p) < 0:
                vertical_v = vertical_v * -1
            module = np.sqrt(np.sum(np.square(vertical_v[0]) + np.square(vertical_v[1])))
            vertical_v = vertical_v / module
            for i in range(1, width + 1):
                states.append([available_points[index][0] + vertical_v[0]*i, available_points[index][1] + vertical_v[1]*i])
            states_list.append(np.array(states))

    # plt.imshow(track)
    # # for index, item in enumerate(contours):
    # #     plt.scatter(item[0], item[1])
    # for states in states_list:
    #     for state in states:
    #         plt.scatter(state[0], state[1])
    #         plt.pause(0.05)
    # plt.show()
    return states_list




if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    # IMG_PATH = "./tracks/line.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    car = PointCar(150, 200)
    fig = plt.figure()

    valid_traj = find_valid_trajectory(car, track)
    print(np.array(valid_traj).shape)
