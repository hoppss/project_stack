import re
import os
import numpy as np
import matplotlib.pyplot as plt

# sphere
Q = np.eye(4)
Q[3, 3] = -(2.13036/2)**2
dualQ = np.linalg.inv(Q)
Twq = np.eye(4)
Twq[0, 3] = 0.446472
Twq[1, 3] = -0.484614
Twq[2, 3] = 0.5
dualQ = Twq.dot(dualQ).dot(Twq.transpose())
print "dualQ=\n", dualQ

# intrinsic, Tcw
K = np.array([[277.19135641132203, 0.0, 160.5], [0.0, 277.19135641132203, 120.5], [0.0, 0.0, 1.0]])


def rotateTwc(Twc):
    # rotate camera coordinate system to get final Twc
    Rx = np.array([[1, 0, 0, 0],
                   [0, 0, 1, 0],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]])
    # Rx = np.array([[1, 0, 0, 0],
    #                [0, np.cos(np.pi/2), np.sin(np.pi/2), 0],
    #                [0, -np.sin(np.pi/2), np.cos(np.pi/2), 0],
    #                [0, 0, 0, 1]])
    Ry = np.array([[0, 0, 1, 0],
                   [0, 1, 0, 0],
                   [-1, 0, 0, 0],
                   [0, 0, 0, 1]])
    # Ry = np.array([[np.cos(np.pi/2), 0, np.sin(np.pi/2), 0],
    #                [0, 1, 0, 0],
    #                [-np.sin(np.pi/2), 0, np.cos(np.pi/2), 0],
    #                [0, 0, 0, 1]])
    Twc = Twc.dot(Rx).dot(Ry)
    return Twc


def projectQuadric(K, Twc):
    # project sphere
    Tcw = np.linalg.inv(Twc)
    P = K.dot(Tcw[:-1, :])
    dualC = P.dot(dualQ).dot(P.transpose())
    C = np.linalg.inv(dualC)
    # draw conic
    x = np.arange(0, 320, 0.1)
    y = np.arange(0, 240, 0.1)
    x, y = np.meshgrid(x, y)
    plt.contour(x, y, C[0,0]*x**2 + 2*C[0,1]*x*y + C[1,1]*y**2 + 2*C[0,2]*x + 2*C[1,2]*y + C[2,2], [0])
    plt.axis('scaled')
    plt.xlim([0, 320])
    plt.ylim([0, 240])
    plt.show()


with open(os.path.join(os.path.dirname(__file__), "Twc_without_rotate.txt"), "r") as f:
    liTwc = []
    count = 1
    for fl in f.readlines():
        if not fl == "\n":
            try:
                liTwc.append([float(s) for s in re.split(r"\s*", fl.strip(" \t\n"))])
            except ValueError:
                liTwc = []
        else:
            liTwc = []
        if len(liTwc) == 4:
            print "***************", count, "***************"
            count = count + 1
            Twc = np.array(liTwc)
            Twc = rotateTwc(Twc)
            print "Twc=\n", Twc
            projectQuadric(K, Twc)
            liTwc = []
