import math
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

log = open("/home/samuele/graal_ws/oal/build/CL_build/WPlogN.txt", "r")
# log = open("log.txt", "r")
Lines = log.readlines()
data = {}

# file = log.read()
for line in Lines:
    line = line.strip()
    # print(line)
    Values = line.split("_")
    # print(Values)
    key = Values[0]
    if key == "Start":
        startPos = (float(Values[1]), float(Values[2]))
    elif key == "Goal":
        goalPos = (float(Values[1]), float(Values[2]))
    elif key == "Time":
        timestamp = float(Values[1])
        data[timestamp] = []
    elif key == "Waypoint":
        data[timestamp].append({"wp": (float(Values[1]), float(Values[2]))})
    elif key == "Obs":
        id = Values[1]
        vxs = []
    elif key == "Position":
        position = (float(Values[1]), float(Values[2]))
    elif key == "Heading":
        theta = float(Values[1])
    elif key == "Vel":
        theta_vel = float(Values[1])
    elif key == "Safety":
        safety = (float(Values[1]), float(Values[2]), float(Values[3]), float(Values[4]))
    elif key == "Max":
        max = (float(Values[1]), float(Values[2]), float(Values[3]), float(Values[4]))
    elif key == "Vx":
        vx = (float(Values[1]), float(Values[2]))
        vxs.append(vx)
    elif key == "Dimx":
        dimx = float(Values[1])
    elif key == "Dimy":
        dimy = float(Values[1])
    elif key == "-":
        data[timestamp].append(
            {"obs": id, "position": position, "heading": theta, "vel_dir": theta_vel, "safety": safety, "max": max,
             "vx1": vxs[0], "vx2": vxs[1],
             "vx3": vxs[2], "vx4": vxs[3], "dimx": dimx, "dimy": dimy})

bk = data
fig = plt.figure(figsize=(10, 5))

timeinstants = sorted(data.keys())

wps = []
count = 0
for timestamp in timeinstants:
    data = bk
    count = count + 1
    ax = fig.add_subplot(1, len(data.keys()), count)
    ax.set_title('Time: ' + str(timestamp))
    # plt.xlim(7, 13)
    # plt.ylim(0, 5)

    wp = data[timestamp][0]
    data[timestamp].remove(wp)
    pol_data = data[timestamp]

    ax.scatter(startPos[0], startPos[1], color='black', marker='o', s=50)
    ax.text(startPos[0] - 0.6, startPos[1], "Start", ha='center', va='center', fontsize=7)
    ax.scatter(goalPos[0], goalPos[1], color='green', marker='x', s=50)
    ax.text(goalPos[0] - 0.6, goalPos[1], "Goal", ha='center', va='center', fontsize=7)

    wp_pos = wp["wp"]
    wps.append(wp_pos)
    trace_x = []
    trace_y = []
    for wp in wps:
        trace_x.append(wp[0])
        trace_y.append(wp[1])

    ax.plot(trace_x, trace_y, 'r.:')
    ax.scatter(wp_pos[0], wp_pos[1], color='red', marker='o')

    # Add polygons and their names to the plot
    for polygon_data in pol_data:
        # print(polygon_data)
        heading = float(polygon_data['heading'])
        vel_dir = float(polygon_data['vel_dir'])
        dim_x = float(polygon_data['dimx'])
        dim_y = float(polygon_data['dimy'])
        position = (float(polygon_data['position'][0]), float(polygon_data['position'][1]))
        safety = (float(polygon_data['safety'][0]), float(polygon_data['safety'][1]), float(polygon_data['safety'][2]),
                  float(polygon_data['safety'][3]))
        max = (float(polygon_data['max'][0]), float(polygon_data['max'][1]), float(polygon_data['max'][2]),
               float(polygon_data['max'][3]))

        vx1 = (position[0] + dim_x / 2 * math.cos(heading) + dim_y / 2 * math.sin(heading),
               position[1] - dim_y / 2 * math.cos(heading) + dim_x / 2 * math.sin(heading))
        vx2 = (position[0] + dim_x / 2 * math.cos(heading) - dim_y / 2 * math.sin(heading),
               position[1] + dim_y / 2 * math.cos(heading) + dim_x / 2 * math.sin(heading))
        vx3 = (position[0] - dim_x / 2 * math.cos(heading) + dim_y / 2 * math.sin(heading),
               position[1] - dim_y / 2 * math.cos(heading) - dim_x / 2 * math.sin(heading))
        vx4 = (position[0] - dim_x / 2 * math.cos(heading) - dim_y / 2 * math.sin(heading),
               position[1] + dim_y / 2 * math.cos(heading) - dim_x / 2 * math.sin(heading))
        hull = [vx1, vx2, vx3, vx4]

        vx1 = (position[0] + max[0] * dim_x / 2 * math.cos(heading) + max[2] * dim_y / 2 * math.sin(heading),
               position[1] - max[2] * dim_y / 2 * math.cos(heading) + max[0] * dim_x / 2 * math.sin(heading))
        vx2 = (position[0] + max[0] * dim_x / 2 * math.cos(heading) - max[3] * dim_y / 2 * math.sin(heading),
               position[1] + max[3] * dim_y / 2 * math.cos(heading) + max[0] * dim_x / 2 * math.sin(heading))
        vx3 = (position[0] - max[1] * dim_x / 2 * math.cos(heading) + max[2] * dim_y / 2 * math.sin(heading),
               position[1] - max[2] * dim_y / 2 * math.cos(heading) - max[1] * dim_x / 2 * math.sin(heading))
        vx4 = (position[0] - max[1] * dim_x / 2 * math.cos(heading) - max[3] * dim_y / 2 * math.sin(heading),
               position[1] + max[3] * dim_y / 2 * math.cos(heading) - max[1] * dim_x / 2 * math.sin(heading))
        mverts = [vx1, vx2, vx3, vx4]

        vx1 = (position[0] + safety[0] * dim_x / 2 * math.cos(heading) + safety[2] * dim_y / 2 * math.sin(heading),
               position[1] - safety[2] * dim_y / 2 * math.cos(heading) + safety[0] * dim_x / 2 * math.sin(heading))
        vx2 = (position[0] + safety[0] * dim_x / 2 * math.cos(heading) - safety[3] * dim_y / 2 * math.sin(heading),
               position[1] + safety[3] * dim_y / 2 * math.cos(heading) + safety[0] * dim_x / 2 * math.sin(heading))
        vx3 = (position[0] - safety[1] * dim_x / 2 * math.cos(heading) + safety[2] * dim_y / 2 * math.sin(heading),
               position[1] - safety[2] * dim_y / 2 * math.cos(heading) - safety[1] * dim_x / 2 * math.sin(heading))
        vx4 = (position[0] - safety[1] * dim_x / 2 * math.cos(heading) - safety[3] * dim_y / 2 * math.sin(heading),
               position[1] + safety[3] * dim_y / 2 * math.cos(heading) - safety[1] * dim_x / 2 * math.sin(heading))
        sverts = [vx1, vx2, vx3, vx4]

        vx1 = polygon_data['vx1']
        vx2 = polygon_data['vx2']
        vx3 = polygon_data['vx3']
        vx4 = polygon_data['vx4']
        verts = [vx1, vx2, vx3, vx4]

        center = np.mean(hull, axis=0)
        sorted_points = sorted(hull, key=lambda p: np.arctan2(p[1] - center[1], p[0] - center[0]))
        poly = Polygon(sorted_points, facecolor='k', edgecolor='k')
        ax.add_patch(poly)
        centroid_x = sum(x for x, y in hull) / len(hull)
        centroid_y = sum(y for x, y in hull) / len(hull)

        center = np.mean(verts, axis=0)
        sorted_points = sorted(verts, key=lambda p: np.arctan2(p[1] - center[1], p[0] - center[0]))
        poly = Polygon(sorted_points, facecolor='None', edgecolor='b')
        ax.add_patch(poly)

        # verts = polygon_data['vxs']
        center = np.mean(sverts, axis=0)
        sorted_points = sorted(sverts, key=lambda p: np.arctan2(p[1] - center[1], p[0] - center[0]))
        poly = Polygon(sorted_points, facecolor='None', edgecolor='r')
        ax.add_patch(poly)
        # verts = polygon_data['vxs']
        center = np.mean(mverts, axis=0)
        sorted_points = sorted(mverts, key=lambda p: np.arctan2(p[1] - center[1], p[0] - center[0]))
        poly = Polygon(sorted_points, facecolor='None', edgecolor='g')
        ax.add_patch(poly)

        plt.arrow(centroid_x, centroid_y, dim_x*0.15 * math.cos(heading), dim_x*0.15 * math.sin(heading),
                  width=dim_y*0.05,
                  head_width=dim_y * 0.3,
                  head_length=dim_y * 0.2, color='blue')

        plt.arrow(centroid_x, centroid_y, dim_x*0.15 * math.cos(vel_dir), dim_x*0.15 * math.sin(vel_dir),
                  width=dim_y*0.05,
                  head_width=dim_y * 0.3,
                  head_length=dim_y * 0.2, color='red')

        ax.text(centroid_x - dim_y*1.3* math.sin(heading), centroid_y + dim_y*1.3* math.cos(heading), polygon_data['obs'], ha='center', va='center', fontsize=10)
        ax.axis('equal')
        # ax.axis('square')
        #ax.set_xlim(8, 12)
        ax.set_aspect('equal')

plt.show()

sys.exit()
