import numpy as np
import matplotlib.pyplot as plt


# setting the axes projection as polar
plt.axes(projection = 'polar')

# setting the radius
r = 2

# creating an array containing the
# radian values
rads = [1, 2, 3]

# plotting the circle
for rad in rads:
    plt.polar(rad, r, 'g.')

# display the Polar plot
plt.show()
