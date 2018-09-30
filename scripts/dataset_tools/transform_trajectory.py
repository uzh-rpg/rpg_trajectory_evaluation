#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 18 11:16:38 2018

@author: Jonathan Huber

Transform textfile containing a trajectory using a constant transformation

T_new = T_old * dT

For example, T_eye = T_hand * T_hand_eye.

"""

import os
import numpy as np
import argparse
import math


def quat2dcm(quaternion):
    """Returns direct cosine matrix from quaternion (Hamiltonian, [x y z w])
    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < np.finfo(float).eps * 4.0:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3]),
        (q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3]),
        (q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1])),
        dtype=np.float64)


def dcm2quat(matrix_3x3):
    """Return quaternion (Hamiltonian, [x y z w]) from rotation matrix.
    This algorithm comes from  "Quaternion Calculus and Fast Animation",
    Ken Shoemake, 1987 SIGGRAPH course notes
    (from Eigen)
    """
    q = np.empty((4, ), dtype=np.float64)
    M = np.array(matrix_3x3, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > 0.0:
        t = math.sqrt(t+1.0)
        q[3] = 0.5*t
        t = 0.5/t
        q[0] = (M[2, 1] - M[1, 2])*t
        q[1] = (M[0, 2] - M[2, 0])*t
        q[2] = (M[1, 0] - M[0, 1])*t
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = math.sqrt(M[i, i] - M[j, j] - M[k, k] + 1.0)
        q[i] = 0.5*t
        t = 0.5/t
        q[3] = (M[k, j] - M[j, k])*t
        q[j] = (M[i, j] + M[j, i])*t
        q[k] = (M[k, i] + M[i, k])*t
    return q


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Analyze trajectories
    ''')
    parser.add_argument('trajectory',
                        help="File containing "
                        "[timestamp tx ty tz qx qy qz qw]")
    parser.add_argument('transformation',
                        help="File with 4x4 transformation matrix, "
                        "Comma separated rows")
    args = parser.parse_args()
    assert os.path.exists(args.trajectory)
    assert os.path.exists(args.transformation)

    out_dir = os.path.dirname(os.path.abspath(args.trajectory))

    # load the file
    f = open(args.trajectory, "r")
    lines = f.readlines()
    timestamp = []
    positions = []
    quats = []
    i = 0
    for x in lines:
        if(i == 0):
            header = x
            i = i+1
            continue
        contents = x.split(' ')
        timestamp.append(contents[0])
        positions.append(np.array(contents[1:4], dtype=float))
        quats.append(np.array(contents[4:8], dtype=float))
    f.close()
    print("Read {0} poses.".format(len(timestamp)))

    T_HE = np.loadtxt(args.transformation, delimiter=',')
    print("Transforamtion to apply:\n{0}".format(T_HE))
    N = len(positions)
    assert(N == len(quats))
    transformed_positions = np.zeros([N, 3])
    transformed_quats = np.zeros([N, 4])
    print('Transforming {0} trajectory poses'.format(N))
    for i in range(N):
        T_old = np.eye(4)
        T_old[0:3, 0:3] = quat2dcm(quats[i])
        T_old[0:3, 3] = positions[i]
        T_new = T_old.dot(T_HE)
        transformed_positions[i, :] = T_new[0:3, 3]
        transformed_quats[i, :] = dcm2quat(T_new[0:3, 0:3])

    file_lines = []
    file_lines.append(header)
    for i in range(N):
        file_lines.append(''.join([str(timestamp[i]), ' ',
                                   str(transformed_positions[i, 0]), ' ',
                                   str(transformed_positions[i, 1]), ' ',
                                   str(transformed_positions[i, 2]), ' ',
                                   str(transformed_quats[i, 0]), ' ',
                                   str(transformed_quats[i, 1]), ' ',
                                   str(transformed_quats[i, 2]), ' ',
                                   str(transformed_quats[i, 3]), '\n']))

    outfn = os.path.join(out_dir, "transformed_poses.txt")
    with open(outfn, 'w') as f:
        f.writelines(file_lines)
    print("Wrote transformed poses to file {0}.".format(outfn))
