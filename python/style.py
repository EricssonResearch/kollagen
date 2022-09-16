#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
style.py: Styling options for matplotlib
"""

import matplotlib.pyplot as plt
import os

script_path = os.path.realpath(__file__)
script_dir  = os.path.dirname(script_path)

# Primary blues
blue1    = "#000082" # Blue 1
blue2    = "#0050ca" # Blue 2
eriblue  = "#0082f0" # Ericsson Blue
blue3    = "#48a8fa" # Blue 3
blue4    = "#85ccff" # Blue 4
# Secondary colors
green    = "#0fc373" # Green
yellow   = "#fad22d" # Yellow
orange   = "#ff8c0a" # Orange
red      = "#ff3232" # Red
purple   = "#af78d2" # Purple
# Neutral tones
black    = "#000000" # Black
gray1    = "#242424" # Gray 1
gray2    = "#767676" # Gray 2
gray3    = "#a0a0a0" # Gray 3
gray4    = "#e0e0e0" # Gray 4
gray5    = "#f2f2f2" # Gray 5
eriwhite = "#fafafa" # Ericsson White
white    = "#ffffff" # White

plt.style.use(script_dir+"/"+"ericsson.mplstyle")
plt.rcParams["font.size"] = 12
plt.rcParams["mathtext.fontset"] = "stix"
plt.rcParams["legend.facecolor"] = "white"
plt.rcParams["figure.facecolor"] = "white"
plt.rcParams["savefig.facecolor"] = "None"
plt.rcParams["patch.edgecolor"] = "k"
plt.rcParams["svg.fonttype"] = "none"

if __name__ == "__main__":
    a1 = [i*1 for i in range(10)]
    a2 = [i*2 for i in range(10)]
    a3 = [i*3 for i in range(10)]
    a4 = [i*4 for i in range(10)]
    a5 = [i*5 for i in range(10)]
    a6 = [i*6 for i in range(10)]
    plt.plot(a1)
    plt.plot(a2)
    plt.plot(a3)
    plt.plot(a4)
    plt.plot(a5)
    plt.plot(a6)
    plt.show()
