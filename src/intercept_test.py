import numpy as np
from numpy import linalg as LA
from math import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from random import random
from scipy.signal import argrelextrema

OBS_TIME = 40
step = pi / 250


def main():
    # Create a figure with two subplots
    fig = plt.figure(figsize=(10, 5))
    # Add the first subplot (3D plot)
    ax = fig.add_subplot(121, projection='3d')
    # Add the second subplot (2D plot)
    ax2 = fig.add_subplot(122)

    # Pick random configuration until it finds desired one
    while (1):
        # TS
        Origin = np.array([random() * 10 - 5, random() * 10 - 5, 0])
        Speed = random() * 5 + 0.5
        Heading = 2 * pi * random() - pi
        Direction = np.array([Speed * cos(Heading), Speed * sin(Heading), 1])
        # OS
        OS_Speed = random() * 5 + 0.5

        points = getIntersectionTime_new(Origin, Direction, OS_Speed)

        if len(points) > 0:
            # Desired config
            """OS_Speed >= Speed and len(points) > 1 and"""
            all_positives = True
            for p in points:
                if p[0] < 0:
                    all_positives = False
            if all_positives:
                break
        else:
            print("NO")

    ax = SetUpPlot(Origin, OS_Speed, Direction, fig, ax)
    # Print data
    print("\nTS \n Origin: ", Origin[0], " ", Origin[1], "\n Heading: ", Heading, "/", round(Heading/pi*180,2), "\n Speed: ", Speed)
    print("OS \n Speed: ", OS_Speed, '\n')

    print("\nNew method results")
    print(" Cone interception in ", len(points), " points.")
    count = 0
    for point in points:
        ax = PlotPath(ax, np.array([0, 0, 0]), point[1], count + 1)
        #print(atan2(point[1][1],point[1][0]))
        print_pos(point, OS_Speed, Direction, Origin, False)
        count = count + 1
    poss = np.array([p[1] for p in points])
    ax.set_zlim(np.min(np.concatenate(poss)) - 10, np.max(np.concatenate(poss)) + 10)
    ax.legend()

    # Brute force
    print("\nBrute force results")
    thetas = np.arange(-pi, pi, 0.01)
    dists = np.array([])
    for theta in thetas:
        t = Origin[0] / (OS_Speed * cos(theta) - Speed * cos(Heading))
        distance = distance_t(t, OS_Speed, theta, Direction, Origin)
        dists = np.append(dists, distance)
        ax2.scatter(theta, distance, color='red')
    # Find the indices of local minima in the first values array
    min_dists = argrelextrema(dists, np.less)
    # Extract the corresponding tuples for the local minima
    thetas_min_dists = [thetas[index] for index in min_dists[0]]
    for theta in thetas_min_dists:
        t = Origin[0] / (OS_Speed * cos(theta) - Speed * cos(Heading))
        p = Origin + t * Direction
        #if t<0:
            #print(" atan2 _ ", pi+atan2(p[1], p[0]) % 2*pi)
            # print(" acos _ ", acos((Origin[0]+Speed*cos(Heading))/(OS_Speed*t)))
        point = [t, p, theta]
        print_pos(point, OS_Speed, Direction, Origin, True)

    ax.set_title('OS speed cone - TS interception')
    ax2.set_title('Distance TS-OS at interception time for each theta')
    ax2.set_xlabel("theta")
    ax2.set_ylabel("distance")
    ax2.set_ylim([0, 10])

    # Old method
    print("\nOld method results")
    points = getIntersectionTime_old(Origin, Heading, Speed, OS_Speed)
    for point in points:
        print_pos(point, OS_Speed, Direction, Origin, True)

    txt = """\n\nThe different thetas, when time is negative, depends on the meaning of a negative time. 
    TS has a fixed heading, so negative time means a position found with negative velocity.
    OS instead,
        - Could have had a heading useful to intercept TS said time ago (theta found with cone)
        - Could intercept, backwards, TS with negative velocity (theta found with brute force)"""
    print(txt)
    # fig.text(.1, .1, "fsfafa")
    # Adjust layout for better visualization
    plt.tight_layout()
    # Show the plots
    plt.show()


def print_pos(point, OS_Speed, Direction, Origin, booool):
    t = point[0]
    p = point[1]
    theta = point[2]
    if booool:
        distance = distance_t(t, OS_Speed, theta, Direction, Origin)
        print("     Time: ", round(t, 2), " theta: ", round(theta, 2), "distance: ", round(distance, 2), " (",
              p[0], ",", p[1], ")")
    else:
        print("     Time: ", round(t, 2), " theta: ", round(theta, 2), " (",
              p[0], ",", p[1], ")")

def distance_t(t, OS_Speed, theta, Direction, Origin):
    p_OS = t * np.array([OS_Speed * cos(theta), OS_Speed * sin(theta), 1])
    p_TS = Origin + Direction * t
    return LA.norm(p_OS - p_TS)


def getIntersectionTime_old(Origin, Heading, Speed, OS_Speed):
    v1 = OS_Speed
    v2 = Speed
    h2 = Heading
    H = Origin[1]
    L = Origin[0]

    k = atan2(-H, L)
    asin_arg = v2 / v1 * sin(h2 + k)
    theta1 = asin(asin_arg) - k
    theta2 = pi - k - asin(asin_arg) - k
    theta2 = theta2 % pi
    t1 = L / (v1 * cos(theta1) - v2 * cos(h2))
    t2 = L / (v1 * cos(theta2) - v2 * cos(h2))

    p = np.array([L, H]) + t1 * np.array([v2 * cos(h2), sin(h2)])
    ipoint = [t1, p, theta1]
    p = np.array([L, H]) + t2 * np.array([v2 * cos(h2), sin(h2)])
    ipoint2 = [t2, p, theta2]

    return [ipoint, ipoint2]


def getIntersectionTime_new(Origin, Direction, OS_Speed):
    gamma = sqrt(OS_Speed ** 2 + 1) / (OS_Speed ** 2 + 1)
    c2 = 1 - gamma ** 2 * np.dot(Direction, Direction)
    c1 = -gamma ** 2 * np.dot(Direction, Origin)
    c0 = -gamma ** 2 * np.dot(Origin, Origin)
    delta = c1 ** 2 - c0 * c2

    if c2 != 0:
        """if delta < 0:
            print("     No intersections")
        el"""
        if delta == 0:
            t = (-c1 + sqrt(delta)) / c2
            p = Origin + Direction * t
            ipoint = [t, p, atan2(p[1], p[0])]
            # print("     Theta: ", atan2(p[1], p[0]), "time: ", t, " (", p[0], ",", p[1], ")")
            return [ipoint, [0, np.array([0, 0, 0]), 0]]
        elif delta > 0:
            t1 = (-c1 + sqrt(delta)) / c2
            t2 = (-c1 - sqrt(delta)) / c2
            p = Origin + Direction * t1
            ipoint1 = [t1, p, atan2(p[1], p[0])]
            dis = distance_t(t1, OS_Speed, atan2(p[1], p[0]), Direction, Origin)

            p = Origin + Direction * t2
            ipoint2 = [t2, p, atan2(p[1], p[0])]
            dis = distance_t(t2, OS_Speed, atan2(p[1], p[0]), Direction, Origin)

            return [ipoint1, ipoint2]
    elif c2 == 0:
        if c1 != 0:
            t = -c0 / (2 * c1)
            p = Origin + Direction * t
            ipoint = [t, p, atan2(p[1], p[0])]
            # print("     Theta: ", atan2(p[1], p[0]), "time: ", t, " (", p[0], ",", p[1], ")")
            return [ipoint, [0, np.array([0, 0, 0]), 0]]
        else:
            print(" wtf, ts and os coincide")

    return []


def SetUpPlot(TS, speed, vel_vect, fig, ax):
    # Plot the 3D subplot
    ax = fig.add_subplot(121, projection='3d')

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    ax.scatter(TS[0], TS[1], TS[2], label='obs origin')
    ax.scatter(0, 0, 0, label='vh origin')

    ax = PlotPath(ax, TS, TS + vel_vect * OBS_TIME, 'obs path')

    line_x, line_y, line_z = zip(TS, TS - vel_vect * OBS_TIME)
    ax.plot(line_x, line_y, line_z, color='c')

    ax = ConePlot(ax, speed)

    # Set labels for the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('t')
    return ax


def PlotPath(ax, p1, p2, name):
    line_x, line_y, line_z = zip(p1, p2)
    ax.plot(line_x, line_y, line_z, label=name)
    ax.scatter(p2[0],p2[1],p2[2])
    return ax


def ConePlot(ax, speed):
    theta = pi / 2 - atan2(1, speed)
    for a in range(0, int(2 * pi / step)):
        # Calculate the current angle
        angle = a * step
        p2 = np.array([0, 0, speed * OBS_TIME])
        p2 = y_rotation(p2, -theta)
        p2 = z_rotation(p2, angle)
        line_x, line_y, line_z = zip(np.array([0, 0, 0]), p2)
        ax.plot(line_x, line_y, line_z, '-r', alpha=0.2)
        line_x, line_y, line_z = zip(np.array([0, 0, 0]), -p2)
        ax.plot(line_x, line_y, line_z, '-', color='grey', alpha=0.2)
    return ax


def x_rotation(vector, theta):
    """Rotates 3-D vector around x-axis"""
    R = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    return np.dot(R, vector)


def y_rotation(vector, theta):
    """Rotates 3-D vector around y-axis"""
    R = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
    return np.dot(R, vector)


def z_rotation(vector, theta):
    """Rotates 3-D vector around z-axis"""
    R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    return np.dot(R, vector)


main()
