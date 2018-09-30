#!/usr/bin/env python2

import os
import numpy as np
import trajectory_utils

import associate_timestamps as associ


def load_stamped_dataset(results_dir, max_diff=0.02):
    '''
    read synchronized estimation and groundtruth and associate the timestamps
    '''
    print('loading dataset in '+results_dir)   

    fn_es = os.path.join(results_dir, 'stamped_traj_estimate.txt')
    fn_gt = os.path.join(results_dir, 'stamped_groundtruth.txt')
    fn_matches = os.path.join(results_dir, 'stamped_est_gt_matches.txt')

    matches = associ.read_files_and_associate(fn_es, fn_gt, 0.0, max_diff)
    dict_matches = dict(matches)
    np.savetxt(fn_matches, np.array(matches, dtype=int), fmt='%d')

    data_es = np.loadtxt(fn_es)
    data_gt = np.loadtxt(fn_gt)

    p_es = []
    p_gt = []
    q_es = []
    q_gt = []
    t_gt = []
    for es_id, es in enumerate(data_es):
        if es_id in dict_matches:
            gt = data_gt[dict_matches[es_id]]
            p_es.append(es[1:4])
            p_gt.append(gt[1:4])
            q_es.append(es[4:8])
            q_gt.append(gt[4:8])
            t_gt.append(gt[0])    
    p_es = np.array(p_es)
    p_gt = np.array(p_gt)
    q_es = np.array(q_es)
    q_gt = np.array(q_gt)
    t_gt = np.array(t_gt)

    return t_gt, p_es, q_es, t_gt, p_gt, q_gt
