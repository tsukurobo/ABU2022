#!/usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Ellipse

if __name__ == "__main__":
    f = open("/home/robosa/abu2022/src/autonomous_driving/data/ndt.txt", "r")
    lines = f.readlines()

    ax = plt.subplot()

    for l in lines:
        elem = l.split(" ")
        sig_x = float(elem[2])
        sig_y = float(elem[3])
        sig_xy = float(elem[4])
        
        if np.fabs(sig_x)>1.0e-12 or np.fabs(sig_y)>1.0e-12 or np.fabs(sig_x)>1.0e-12:
            print("x: {}, y: {}, sig_x: {}, sig_y: {}, sig_xy: {}".format(float(elem[0]), float(elem[1]), sig_x, sig_y, sig_xy))
            cov = np.array([[sig_x, sig_xy], [sig_xy, sig_y]])
            lamdas, vecs = np.linalg.eig(cov)

            theta = np.arctan2(vecs[0, 1], vecs[0, 0])

            el = Ellipse(xy=[float(elem[0]), float(elem[1])],
                     width=50*lamdas[0], height=50.0*lamdas[1],
                     angle=theta)

            ax.add_patch(el)
            ax.autoscale()

    plt.show()