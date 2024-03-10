import Trajectory_Generator as tg
import numpy as np
from math import pi

dt = 0.1
# dt = 0.02

def main():
    joints = []
    joints.append({'ID': 7, 'IP': '10.10.10.8', 'viaPoints': []})
    
    joints[0]['viaPoints'].append(0.0)
    joints[0]['viaPoints'].append(0.5)

    via = []
    via.append(np.array([0.0, 0.0, 0.0]))
    via.append(np.array([1.0, 5.0, 3.0]))
    via.append(np.array([2.0, 7.0, -3.0]))

    print(via[1][0])

    print(joints[0]['viaPoints'][1])

    # time, q0, q1, q2...
    viaPoints = np.array([[0.0, 0.0, 1.0],
                          [1.0, 5.0, -7.3],
                          [2.0, 0.0, 2.0]]).T

    vp = np.zeros([len(via), len(via[0])])
    for i in range(len(via)) :
        vp[i] = via[i]

    print(via)
    print(vp)

    [posTraj, velTraj] = tg.getTraj(viaPoints, dt, Kv = 0.0)

    print(posTraj.T)

if __name__ == '__main__':
    main()