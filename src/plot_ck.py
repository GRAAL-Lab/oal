import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    # ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


log = open("/home/samuele/graal_ws/oal/build/CL_build/CKlog.txt", "r")
# log = open("CKlog.txt", "r")
Lines = log.readlines()

plots = []
plot = []
for line in Lines:
    line = line.strip()
    Values = line.split("_")

    if Values[0] == "---":
        plots.append(plot)

        plot = []
    else:
        plot.append(line)
plots.append(plot)

for plot in plots:
    if plot == []:
        continue
    # file = log.read()
    isToPlot = False
    isGood = False

    data = {}

    for line in plot:
        # line = line.strip()
        # print(line)
        Values = line.split("_")
        # print(Values)
        key = Values[0]
        if key == "Start":
            startPos = (float(Values[1]), float(Values[2]), float(Values[3]))
        elif key == "Goal":
            goalPos = (float(Values[1]), float(Values[2]), float(Values[3]))
        elif key == "Obs":
            id = Values[1]
            data[id] = []
            vxs = []
            isToPlot = False
        elif key == "Direction":
            dir = (float(Values[1]), float(Values[2]), float(Values[3]))

        elif key == "Vx":
            vx = (float(Values[1]), float(Values[2]))
            vxs.append(vx)
        elif key == "-":
            data[id].append({"dir": dir, "vxs": vxs})
        elif key == "PlotIt":
            isToPlot = True
        elif key == "Good":
            isGood = True

    if (not isToPlot or not isGood):
        continue
    else:
        print(".")
        isToPlot = False
        isGood = False
    bk = data
    fig = plt.figure(figsize=(10, 5))

    # ax = fig.add_subplot(1, len(plots), 1)
    ax = fig.add_subplot(projection="3d")
    # ax.set_title('Time: '+timestamp)

    x, y, z = [startPos[0], goalPos[0]], [startPos[1], goalPos[1]], [startPos[2], goalPos[2]]
    ax.scatter(x, y, z, c='red')  # , s=100)
    ax.plot(x, y, z, color='black')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Time')
    obs = data.keys()

    for ob in obs:
        obstacle = data[ob][0]
        dir = obstacle['dir']
        vxs = obstacle['vxs']

        vxs3d_start = []
        vxs3d_goal = []
        for vx in vxs:
            # start time
            vx_ = [vx[0], vx[1], 0]
            vxs3d_start.append(vx_)
            # goal time
            vx_next = [vx[0] + dir[0] * goalPos[2], vx[1] + dir[1] * goalPos[2], dir[2] * goalPos[2]]
            vxs3d_goal.append(vx_next)
        last = vxs3d_start[3]
        vxs3d_start[3] = vxs3d_start[2]
        vxs3d_start[2] = last
        last = vxs3d_goal[3]
        vxs3d_goal[3] = vxs3d_goal[2]
        vxs3d_goal[2] = last
        vertices = vxs3d_start + vxs3d_goal
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[1], vertices[2], vertices[6], vertices[5]],
            [vertices[2], vertices[3], vertices[7], vertices[6]],
            [vertices[0], vertices[3], vertices[7], vertices[4]],
            [vertices[4], vertices[5], vertices[6], vertices[7]]
        ]

        ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.6))

    set_axes_equal(ax)
    plt.show()
