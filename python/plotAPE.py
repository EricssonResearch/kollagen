#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
plot.py: Plot results
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

X = np.fromfile("APE_time.bin", dtype=np.intc)
Y = []

N = 100
rolling_average = True

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
        Y.append(np.fromfile("./APE_agent"+str(i)+".bin", dtype=np.double))
    except:
        OK = False

for i in range(0,len(Y)):
    if rolling_average:
        plt.plot(X[N-1:], np.convolve(Y[i], np.ones(N)/N, mode='valid'), label="Agent %i" % (i+1))
    else:
        plt.plot(X, Y[i], label="Agent %i" % (i+1))

plt.legend()

if filename:
    plt.savefig(filename)
else:
    plt.show()
