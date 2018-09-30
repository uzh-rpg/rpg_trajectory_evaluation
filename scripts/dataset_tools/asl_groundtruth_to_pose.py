#!/usr/bin/env python2

import os
import argparse

import numpy as np


def extract(gt, out_filename):
    fout = open(out_filename, 'w')
    fout.write('# timestamp tx ty tz qx qy qz qw\n')

    n = 0
    with open(gt, 'rb') as fin:
        data = np.genfromtxt(fin, delimiter=",")
        for l in data:
            fout.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                       (l[0]/1e9, l[1], l[2], l[3], l[5], l[6], l[7], l[4]))
        n = n + 1
    print('wrote ' + str(n) + ' poses to the file: ' + out_filename)
    fout.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts pose from a csv file in ASL format to space separated format.
    Quaternion is ordered as [x y z w]
    ''')
    parser.add_argument('gt', help='Ground truth csv')
    parser.add_argument('--output', default='groundtruth.txt')
    args = parser.parse_args()

    output_fn = os.path.join(os.path.dirname(os.path.abspath(args.gt)),
                             args.output)

    print('Extract ground truth pose from file '+args.gt)
    print('Saving to file '+output_fn)
    extract(args.gt, output_fn)
