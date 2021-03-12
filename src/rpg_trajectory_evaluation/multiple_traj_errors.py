#!/usr/bin/env python2

import numpy as np
import os
import pickle

import results_writer as rw
from metrics import kRelMetrics, kRelMetricLables


class MulTrajError(object):
    kAbsMetrics = ['rmse_trans', 'rmse_rot', 'rmse_scale']
    kAbsMetricLabels = ['trans', 'rot', 'scale']

    def __init__(self):
        self.abs_errors = {'rmse_trans': [], 'rmse_rot': [],
                           'rmse_scale': []}
        self.rmse_trans = []
        self.rmse_rot = []
        self.rmse_scale = []
        self.rel_errors = {}
        self.keys = self.rel_errors.keys()
        self.n_traj = 0
        self.save_results_dir = None
        self.cache_results_dir = None
        self.align_str = None
        self.rel_distances = None
        self.success_indices = []
        # useful information for analysis
        self.uid = None
        self.alg = None
        self.dataset = None

    def addTrajectoryError(self, traj, index):
        if not traj.abs_errors:
            print("Absolute errors are not available.")
        else:
            self.abs_errors['rmse_trans'].append(
                (traj.abs_errors['abs_e_trans_stats'])['rmse'])
            self.abs_errors['rmse_rot'].append(
                (traj.abs_errors['abs_e_rot_stats'])['rmse'])
            self.abs_errors['rmse_scale'].append(
                (traj.abs_errors['abs_e_scale_stats'])['rmse'])

        if not self.rel_errors.keys():
            # first run
            for d in traj.preset_boxplot_distances:
                e = traj.rel_errors[d]
                self.rel_errors[d] = {}
                for et in kRelMetrics:
                    (self.rel_errors[d])[et] = e[et]
        else:
            # append
            for d in traj.preset_boxplot_distances:
                e = traj.rel_errors[d]
                assert d in self.rel_errors, "Could not find the distances"
                for et in kRelMetrics:
                    (self.rel_errors[d])[et] = np.concatenate(
                        (self.rel_errors[d][et], e[et]))
        self.n_traj += 1
        self.success_indices.append(index)
        if not self.save_results_dir:
            self.save_results_dir = traj.saved_results_dir
        else:
            assert self.save_results_dir == traj.saved_results_dir

        if not self.cache_results_dir:
            self.cache_results_dir = traj.cache_results_dir
        else:
            assert self.cache_results_dir == traj.cache_results_dir

        if not self.align_str:
            self.align_str = traj.align_str
        else:
            assert self.align_str == traj.align_str
        if not self.rel_distances:
            self.rel_distances = traj.preset_boxplot_distances

    def summary(self):
        print("===> MulTrajError: summarized {0} trajectories.".format(
            self.n_traj))
        print("- Successs indices: {0}".format(self.success_indices))
        if self.n_traj == 0:
            return
        print("Relative errors numbers:")
        for d, e in self.rel_errors.items():
            print("- {0}: {1} {2} samples".format(d, e.keys(),
                                                  e['rel_trans'].size))

    def updateStatistics(self):
        if self.n_traj == 0:
            return
        for et in self.kAbsMetrics:
            self.abs_errors[et+'_stats'] = rw.compute_statistics(
                np.array(self.abs_errors[et]))

        self.overall_rel_errors = {}
        for et in kRelMetrics:
            values = []
            for d in self.rel_errors:
                self.rel_errors[d][et+'_stats'] = rw.compute_statistics(
                    self.rel_errors[d][et])
                values.extend(self.rel_errors[d][et].tolist())
            self.overall_rel_errors[et] = rw.compute_statistics(values)

    def saveErrors(self):
        if self.n_traj == 0:
            return
        abs_fn = os.path.join(self.save_results_dir,
                              'mt_abs_err_'+self.align_str+'.yaml')
        for et, label in zip(self.kAbsMetrics, self.kAbsMetricLabels):
            rw.update_and_save_stats(self.abs_errors[et+'_stats'],
                                     label, abs_fn)
            np.savetxt(
                os.path.join(self.save_results_dir,
                             'mt_' + et + '_all_'+self.align_str+'.txt'),
                np.array(self.abs_errors[et]))

        for dist in self.rel_errors:
            cur_err = self.rel_errors[dist]
            dist_str = "{:3.1f}".format(dist).replace('.', '_')
            dist_fn = os.path.join(
                self.save_results_dir, 'mt_rel_err_'+dist_str + '.yaml')
            for et, label in zip(kRelMetrics, kRelMetricLables):
                rw.update_and_save_stats(cur_err[et+'_stats'], label, dist_fn)

        overall_rel_fn = os.path.join(
            self.save_results_dir, 'mt_rel_err_overall' + '.yaml')
        for et, label in zip(kRelMetrics, kRelMetricLables):
            rw.update_and_save_stats(self.overall_rel_errors[et], label, overall_rel_fn)

        np.savetxt(
                os.path.join(
                    self.save_results_dir, 'mt_success_indices'+'.txt'),
                np.array(self.success_indices).astype(int), fmt='%i')

    def cache_current_error(self):
        if self.abs_errors:
            with open(os.path.join(self.cache_results_dir,
                                   'mt_cached_abs_err_' +
                                   self.align_str+'.pickle'), 'wb') as f:
                pickle.dump(self.abs_errors, f)
        if self.rel_errors:
            with open(os.path.join(self.cache_results_dir,
                                   'mt_cached_rel_err.pickle'), 'wb') as f:
                pickle.dump(self.rel_errors, f)

    def get_relative_errors_and_distances(
            self, error_types=['rel_trans', 'rel_trans_perc', 'rel_yaw']):
        rel_errors = {}
        for err_i in error_types:
            assert err_i in kRelMetrics
            rel_errors[err_i] = [[self.rel_errors[d][err_i]
                                 for d in self.rel_distances]]
        return rel_errors, self.rel_distances
