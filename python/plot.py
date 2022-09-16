#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
plot.py: Plot results
"""

import sys
import style
import numpy as np
import matplotlib.pyplot as plt

X = []
Y = []

try:
    filename = sys.argv[1]
    if filename[-4:] != ".svg":
        filename += ".svg"
except:
    filename = ""


OK = True
i = 0
while OK:
    i += 1
    try:
        X.append(np.fromfile("./x"+str(i)+".bin", dtype=np.double))
        Y.append(np.fromfile("./y"+str(i)+".bin", dtype=np.double))
    except:
        OK = False

for i in range(0,len(X)):
    plt.plot(X[i],Y[i],".-", linewidth=0.5, markersize=1.0)

plt.axis("equal")

if filename:
    plt.savefig(filename)
else:
    plt.show()
