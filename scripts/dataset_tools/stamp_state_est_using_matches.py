#!/usr/bin/env python2

import os
import argparse

import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='stamp state estimate that are marked using id')
    parser.add_argument('state_est',
                        help='state estimate file that starts with id')
    parser.add_argument('--matches', type=str,
                        help='id pairs: state estimate -> groundtruth',
                        default='groundtruth_matches.txt')
    parser.add_argument('--groundtruth', type=str,
                        help='id, stamp, gt', default='groundtruth.txt')
    args = parser.parse_args()

    assert os.path.exists(args.state_est)
    assert os.path.exists(args.matches)
    assert os.path.exists(args.groundtruth)
    outdir = os.path.dirname(os.path.abspath(args.state_est))
    outfn = os.path.join(outdir, 'stamped_' + os.path.basename(args.state_est))

    print("Going to stamp {0} with {1} and write to {2}".format(args.state_est,
                                                                args.matches,
                                                                outfn))

    id_state_est = np.loadtxt(args.state_est)
    print("Loaded {0} states.".format(id_state_est.shape[0]))

    est_gt_id_map = []
    with open(args.matches) as f:
        content = f.readlines()
        content = [x.strip().split(' ') 
                   for x in content if not x.startswith('#')]
        est_gt_id_map = [(int(l[0]), int(l[1])) for l in content]
    est_gt_id_map = dict(est_gt_id_map)
    print("Loaded {0} id pairs.".format(len(est_gt_id_map)))

    id_stamp_map = []
    with open(args.groundtruth) as f:
        content = f.readlines()
        content = [x.strip().split(' ') 
                   for x in content if not x.startswith('#')]
        id_stamp_map = [(int(l[0]), float(l[1])) for l in content]
    id_stamp_map = dict(id_stamp_map)
    print("Loaded {0} id pairs.".format(len(id_stamp_map)))

    stamped_states = []
    for id_s in id_state_est.tolist():
        cur_id = int(id_s[0])
        if cur_id in est_gt_id_map:
            stamped_s = id_s[:]
            stamped_s[0] = id_stamp_map[est_gt_id_map[cur_id]]
            stamped_states.append(stamped_s)
    
    np.savetxt(outfn, stamped_states, header='time x y z qx qy qz qw')
    print("Found matches and written for {0} states.".format(
        len(stamped_states)))
