#!/usr/bin/env python2

kNsToEstFnMapping = {'traj_est': 'vins_result_no_loop',
                     'pose_graph': 'vins_result_loop',
                     'ba_estimate': 'stamped_ba_estimate'}
kNsToMatchFnMapping = {'traj_est': 'vins_result_no_loop_gt_matches',
                       'pose_graph': 'vins_result_loop_gt_matches',
                       'ba_estimate': 'stamped_ba_gt_matches'}
kFnExt = 'csv'
