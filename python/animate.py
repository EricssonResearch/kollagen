#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
animate.py: Animate results
"""

import sys
import style
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

X = []
Y = []
try:
    filename = sys.argv[1]
    if filename[-4:]!=".mp4":
        filename += ".mp4"
except:
    filename = "animation.mp4"

OK = True
i = 0
while OK:
    i += 1
    try:
        X.append(np.fromfile("./x"+str(i)+".bin", dtype=np.double))
        Y.append(np.fromfile("./y"+str(i)+".bin", dtype=np.double))
    except:
        OK = False

padding = 1

x_max = np.max(X) + padding
x_min = np.min(X) - padding

y_max = np.max(Y) + padding
y_min = np.min(Y) - padding

Writer = animation.writers['ffmpeg']
writer = Writer(fps=60, metadata=dict(artist='Me'), bitrate=1800)

def update(num, paths, positions, T):
    print("Frame", num, "of", T)
    for i, path in enumerate(paths):
        path[0].set_data(X[i][0:num], Y[i][0:num])
    for i, position in enumerate(positions):
        position[0].set_data(X[i][num], Y[i][num])


T = len(X[0])

paths = []
positions = []

fig = plt.figure()
ax = fig.gca()
ax.set_xlim([x_min,x_max])
ax.set_ylim([y_min,y_max])
for i in range(len(X)):
    paths.append(ax.plot([], [], ":C%i" % i,linewidth=0.5),)
    positions.append(ax.plot([], [], "vC%i" % i, markersize=2.0),)

ani = animation.FuncAnimation(fig, update, T, fargs=(paths, positions, T), interval=1, blit=False)

print("Saving animation as %s" % filename)
ani.save(filename, writer=writer,dpi=100)

plt.show()
