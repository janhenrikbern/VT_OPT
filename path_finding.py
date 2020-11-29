import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar

# Set trellis graph
N_STATES = 10
N_NODES = 199


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
    visible = N_NODES
    width = N_STATES

    contours = get_inner_contours(track)

    # horizontal to the left is the original angle
    (x, y), index = find_closest_point(car.location, contours)
    direction = find_direction(car, index, contours) 
    
    if visible <= 0:
        print("invalid visible number")
        exit()

    available_points = np.array([])
    while visible > 0:
        if visible > len(contours):
            single_visible = len(contours)
        else:
            single_visible = visible
        # direction to the whole track
        if direction > 0:
            if index + single_visible*direction > len(contours):
                available_points_temp = np.concatenate((contours[index:len(contours)],contours[0:index + single_visible * direction - len(contours)]), axis=0)
            else:
                available_points_temp = contours[index:index + single_visible*direction]
        else:
            if index + single_visible*direction < 0:
                available_points_temp = np.concatenate((contours[index + single_visible * direction:], contours[0:index]), axis=0)
            else:
                available_points_temp = contours[index + single_visible*direction: index]
            available_points_temp = available_points_temp[::-1]
        if len(available_points) > 0:
            available_points = np.concatenate((available_points, available_points_temp), axis=0)
        else:
            available_points = available_points_temp
        visible = visible - len(contours)
        
    states_list = []
    for index, ap in enumerate(available_points):
        states = []
        if not index == 0 and not index == len(available_points)-1:
            # horizontal to the speed
            horiz_v = np.array([available_points[index-1][0]-available_points[index+1][0], available_points[index-1][1]-available_points[index+1][1]])
            vertical_v = np.array([horiz_v[1], -horiz_v[0]])

            # there are two direction of the vertical of speed. select the one point to outer contour
            if len(states_list) == 0:
                point_to_p = np.array([car.location[0] - available_points[index][0], car.location[1] - available_points[index][1]])
            else:
                point_to_p = np.array([states_list[-1][-1][0] - available_points[index][0], states_list[-1][-1][1] - available_points[index][1]])
            if np.dot(vertical_v, point_to_p) < 0:
                vertical_v = vertical_v * -1

            # moudle is to find points along the vertical vector
            module = np.sqrt(np.sum(np.square(vertical_v[0]) + np.square(vertical_v[1])))
            vertical_v = vertical_v / module

            # get outer contour
            times = 1
            while track[int(available_points[index][1] + vertical_v[1]*times), int(available_points[index][0] + vertical_v[0]*times)]:
                times = times + 1
            vertical_v = vertical_v * (times+1)
            # to remove the head and tail
            for i in range(1, width+1):
                states.append([available_points[index][0] + vertical_v[0]*i/(width+1), available_points[index][1] + vertical_v[1]*i/(width+1)])
            states_list.append(np.array(states))

    # plt.imshow(track)
    # # for index, item in enumerate(contours):
    # #     plt.scatter(item[0], item[1])
    # for states in states_list:
    #     for state in states:
    #         plt.scatter(state[0], state[1])
    #         plt.pause(0.05)
    # plt.show()
    return states_list[::-1]


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    # IMG_PATH = "./tracks/line.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    car = PointCar(150, 200)
    fig = plt.figure()

    valid_traj = find_valid_trajectory(car, track)
    print(np.array(valid_traj).shape)
