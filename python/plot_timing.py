#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
plot_timing.py: Plot timing from timing.cpp.
"""

import style
import numpy as np
import matplotlib.pyplot as plt

plt.figure()
X = []
Y = []

OK = True
i = 0
while OK:
    i += 1
    try:
        X.append(np.fromfile("./timing_x"+str(i)+".bin", dtype=np.intc))
        Y.append(np.fromfile("./timing_y"+str(i)+".bin", dtype=np.double))
    except:
        OK = False

for i in range(0,len(X)):
    plt.loglog(X[i],Y[i],"v--", label="$N_\\mathrm{agents}=%i$" % (i+1))

plt.xlabel("$N_\\mathrm{steps}$")
plt.ylabel("Time [ms]")
plt.legend()

plt.figure()
X2 = []
Y2 = []

OK = True
i = 1
while OK:
    i *= 10
    try:
        X2.append(np.fromfile("./timing_agents_x"+str(i)+".bin", dtype=np.intc))
        Y2.append(np.fromfile("./timing_agents_y"+str(i)+".bin", dtype=np.double))
    except:
        OK = False

for i in range(0,len(X2)):
    plt.loglog(X2[i],Y2[i],"v--", label="$N_\\mathrm{steps}=%.0e$" % 10**(i+1))

plt.figure()
X3 = []
Y3 = []

OK = True
i = 1
while OK:
    i *= 10
    try:
        X3.append(np.fromfile("./timing_radius_x"+str(i)+".bin", dtype=np.intc))
        Y3.append(np.fromfile("./timing_radius_y"+str(i)+".bin", dtype=np.double))
    except:
        OK = False

for i in range(0,len(X3)):
    plt.loglog(X3[i],Y3[i],"v--", label="$N_\\mathrm{steps}=%.0e$" % 10**(i+1))

plt.xlabel("$R_\\mathrm{inter} = R_\\mathrm{intra}$")
plt.ylabel("Time [ms]")
plt.legend()

plt.show()
