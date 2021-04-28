#!/usr/bin/env python2

import os
import yaml
import math
import pickle

import numpy as np
from colorama import init, Fore

import trajectory_utils as traj_utils
import trajectory_loading as traj_loading
import results_writer as res_writer
import compute_trajectory_errors as traj_err
import align_utils as au
from metrics import kRelMetrics, kRelMetricLables

import transformations as tf


class Trajectory:
    rel_error_cached_nm = 'cached_rel_err'
    rel_error_prefix = 'relative_error_statistics_'
    saved_res_dir_nm = 'saved_results'
    cache_res_dir_nm = 'cached'
    default_boxplot_perc = [0.1, 0.2, 0.3, 0.4, 0.5]

    def __init__(self, results_dir, platform='', alg_name='', dataset_name='',
                 align_type='sim3', align_num_frames=-1, suffix='',
                 est_type='traj_est',
                 nm_gt='stamped_groundtruth.txt',
                 nm_est='stamped_traj_estimate.txt',
                 nm_matches='stamped_est_gt_matches.txt',
                 preset_boxplot_distances=[],
                 preset_boxplot_percentages=[]):

        assert os.path.exists(results_dir),\
            "Specified directory {0} does not exist.".format(results_dir)
        assert align_type in ['first_frame', 'sim3', 'se3']

        # information of the results, useful as labels
        self.platform = platform
        self.alg = alg_name
        self.dataset_short_name = dataset_name
        self.uid = self.platform + '_' + self.alg + '_' +\
            self.dataset_short_name
        self.est_type = est_type
        self.suffix_str = self.get_suffix_str(suffix)
        self.success = False

        self.data_dir = results_dir
        self.data_loaded = False
        self.data_aligned = False
        self.saved_results_dir = os.path.join(
            os.path.join(self.data_dir, Trajectory.saved_res_dir_nm),
            self.est_type)
        if not os.path.exists(self.saved_results_dir):
            os.makedirs(self.saved_results_dir)

        self.cache_results_dir = os.path.join(
            self.saved_results_dir, Trajectory.cache_res_dir_nm)
        if not os.path.exists(self.cache_results_dir):
            os.makedirs(self.cache_results_dir)

        self.align_type = align_type
        self.align_num_frames = int(align_num_frames)

        self.eval_cfg = os.path.join(self.data_dir, 'eval_cfg.yaml')

        if os.path.exists(self.eval_cfg):
            print("Find evaluation configuration, will overwrite default.")
            with open(self.eval_cfg, 'r') as f:
                eval_cfg = yaml.load(f, Loader=yaml.FullLoader)
                print("The current evaluation configuration is "
                      "{0}".format(eval_cfg))
                self.align_type = eval_cfg['align_type']
                self.align_num_frames = eval_cfg['align_num_frames']
        self.align_str = self.align_type + '_' + str(self.align_num_frames)

        self.start_end_time_fn = os.path.join(self.data_dir, 'start_end_time.yaml')
        self.start_time_sec = -float('inf')
        self.end_time_sec = float('inf')
        if os.path.exists(self.start_end_time_fn):
            print("Find start end time for evaluation.")
            with open(self.start_end_time_fn, 'r') as f:
                d = yaml.load(f, Loader=yaml.FullLoader)
                if 'start_time_sec' in d:
                    self.start_time_sec = d['start_time_sec']
                if 'end_time_sec' in d:
                    self.end_time_sec = d['end_time_sec']
                print("Will analyze trajectory ranging from {} to {}.".format(
                    self.start_time_sec, self.end_time_sec))

        self.abs_errors = {}

        # we cache relative error since it is time-comsuming to compute
        self.rel_errors = {}
        self.cached_rel_err_fn = os.path.join(
            self.cache_results_dir,
            self.rel_error_cached_nm+self.suffix_str+".pickle")

        print("Loading {0} and {1}...".format(nm_gt, nm_est))
        self.data_loaded = self.load_data(nm_gt, nm_est, nm_matches)
        if not self.data_loaded:
            print(Fore.RED+"Loading data failed.")
            return

        self.boxplot_pcts = preset_boxplot_percentages
        if len(preset_boxplot_distances) != 0:
            print("Use preset boxplot distances.")
            self.preset_boxplot_distances = preset_boxplot_distances
        else:
            if not self.boxplot_pcts:
                self.boxplot_pcts = Trajectory.default_boxplot_perc
            print("Use percentages {} for boxplot.".format(self.boxplot_pcts))
            self.compute_boxplot_distances()

        self.align_trajectory()

    def load_data(self, nm_gt, nm_est, nm_matches):
        """
        Loads the trajectory data. The resuls {p_es, q_es, p_gt, q_gt} is
        synchronized and has the same length.
        """
        if not os.path.exists(os.path.join(self.data_dir, nm_gt)) or \
                not os.path.exists(os.path.join(self.data_dir, nm_est)):
            print(Fore.RED+"Either groundtruth or estimate does not exist")
            return False

        print(Fore.RED+'Loading trajectory data...')

        # only timestamped pose series is supported
        self.t_es, self.p_es, self.q_es, self.t_gt, self.p_gt, self.q_gt =\
            traj_loading.load_stamped_dataset(
                self.data_dir, nm_gt, nm_est,
                os.path.join(Trajectory.saved_res_dir_nm, self.est_type,
                             nm_matches),
                start_t_sec=self.start_time_sec, end_t_sec=self.end_time_sec)
        self.t_gt_raw, self.p_gt_raw, self.q_gt_raw =\
            traj_loading.load_raw_groundtruth(self.data_dir, nm_gt,
                                              start_t_sec=self.start_time_sec,
                                              end_t_sec=self.end_time_sec)
        if self.p_es.size == 0:
            print(Fore.RED+"Empty estimate file.")
            return False
        self.accum_distances = traj_utils.get_distance_from_start(self.p_gt_raw)
        self.traj_length = self.accum_distances[-1]
        self.accum_distances = traj_utils.get_distance_from_start(self.p_gt)

        if os.path.isfile(self.cached_rel_err_fn):
            print('Loading cached relative (odometry) errors from ' +
                  self.cached_rel_err_fn)
            with open(self.cached_rel_err_fn, "rb") as f:
                self.rel_errors = pickle.load(f)
            print("Loaded odometry error calcualted at {0}".format(
                self.rel_errors.keys()))

        print(Fore.GREEN+'...done.')

        return True

    def cache_current_error(self):
        if self.rel_errors:
            with open(self.cached_rel_err_fn, 'wb') as f:
                pickle.dump(self.rel_errors, f)
            print(Fore.YELLOW + "Saved relative error to {0}.".format(
                self.cached_rel_err_fn))

    @staticmethod
    def get_suffix_str(suffix):
        if suffix is not '':
            return "_#"+suffix
        else:
            return suffix

    @staticmethod
    def remove_cached_error(data_dir, est_type='', suffix=''):
        print("To remove cached error in {0}".format(data_dir))
        suffix_str = Trajectory.get_suffix_str(suffix)
        base_fn = Trajectory.rel_error_cached_nm+suffix_str+'.pickle'
        Trajectory.remove_files_in_cache_dir(data_dir, est_type, base_fn)

    @staticmethod
    def _safe_remove_file(abs_rm_fn):
        if os.path.exists(abs_rm_fn):
            os.remove(abs_rm_fn)
            print('Removed {0}'.format(abs_rm_fn))
        else:
            print(Fore.YELLOW + 'Cannot find file {0}'.format(abs_rm_fn))

    @staticmethod
    def remove_files_in_cache_dir(data_dir, est_type, base_fn):
        rm_fn = os.path.join(data_dir, Trajectory.saved_res_dir_nm,
                             est_type, Trajectory.cache_res_dir_nm, base_fn)
        Trajectory._safe_remove_file(rm_fn)

    @staticmethod
    def remove_files_in_save_dir(data_dir, est_type, base_fn):
        rm_fn = os.path.join(data_dir, Trajectory.saved_res_dir_nm,
                             est_type, base_fn)
        Trajectory._safe_remove_file(rm_fn)

    @staticmethod
    def truncate(number, decimals=0):
        """
        Returns a value truncated to a specific number of decimal places.
        """
        if not isinstance(decimals, int):
            raise TypeError("decimal places must be an integer.")
        elif decimals < 0:
            raise ValueError("decimal places has to be 0 or more.")
        elif decimals == 0:
            return math.trunc(number)

        factor = 10.0 ** decimals
        return math.trunc(number * factor) / factor


    def compute_boxplot_distances(self):
        print("Computing preset subtrajectory lengths for relative errors...")
        print("Use percentage {0} of trajectory length.".format(self.boxplot_pcts))
        print("Trajectory length {0}".format(self.traj_length))
        self.preset_boxplot_distances = [self.truncate(pct*self.traj_length, 2)
                                          for pct in self.boxplot_pcts]

        print("...done. Computed preset subtrajecory lengths:"
              " {0}".format(self.preset_boxplot_distances))

    def align_trajectory(self):
        if self.data_aligned:
            print("Trajectory already aligned")
            return
        print(Fore.RED +
              "Aliging the trajectory estimate to the groundtruth...")

        print("Alignment type is {0}.".format(self.align_type))
        n = int(self.align_num_frames)
        if n < 0.0:
            print('To align all frames.')
            n = len(self.p_es)
        else:
            print('To align trajectory using ' + str(n) + ' frames.')

        self.trans = np.zeros((3,))
        self.rot = np.eye(3)
        self.scale = 1.0
        if self.align_type == 'none':
            pass
        else:
            self.scale, self.rot, self.trans = au.alignTrajectory(
                self.p_es, self.p_gt, self.q_es, self.q_gt,
                self.align_type, self.align_num_frames)

        self.p_es_aligned = np.zeros(np.shape(self.p_es))
        self.q_es_aligned = np.zeros(np.shape(self.q_es))
        for i in range(np.shape(self.p_es)[0]):
            self.p_es_aligned[i, :] = self.scale * \
                self.rot.dot(self.p_es[i, :]) + self.trans
            q_es_R = self.rot.dot(
                tf.quaternion_matrix(self.q_es[i, :])[0:3, 0:3])
            q_es_T = np.identity(4)
            q_es_T[0:3, 0:3] = q_es_R
            self.q_es_aligned[i, :] = tf.quaternion_from_matrix(q_es_T)

        self.data_aligned = True
        print(Fore.GREEN+"... trajectory alignment done.")

    def compute_absolute_error(self):
        if self.abs_errors:
            print("Absolute errors already calculated")
        else:
            print(Fore.RED+'Calculating RMSE...')
            # align trajectory if necessary
            self.align_trajectory()
            e_trans, e_trans_vec, e_rot, e_ypr, e_scale_perc =\
                traj_err.compute_absolute_error(self.p_es_aligned,
                                                self.q_es_aligned,
                                                self.p_gt,
                                                self.q_gt)
            stats_trans = res_writer.compute_statistics(e_trans)
            stats_rot = res_writer.compute_statistics(e_rot)
            stats_scale = res_writer.compute_statistics(e_scale_perc)

            self.abs_errors['abs_e_trans'] = e_trans
            self.abs_errors['abs_e_trans_stats'] = stats_trans

            self.abs_errors['abs_e_trans_vec'] = e_trans_vec

            self.abs_errors['abs_e_rot'] = e_rot
            self.abs_errors['abs_e_rot_stats'] = stats_rot

            self.abs_errors['abs_e_ypr'] = e_ypr

            self.abs_errors['abs_e_scale_perc'] = e_scale_perc
            self.abs_errors['abs_e_scale_stats'] = stats_scale
            print(Fore.GREEN+'...RMSE calculated.')
        return

    def write_errors_to_yaml(self):
        self.abs_err_stats_fn = os.path.join(
            self.saved_results_dir, 'absolute_err_statistics'+'_' +
            self.align_str + self.suffix_str + '.yaml')
        res_writer.update_and_save_stats(
            self.abs_errors['abs_e_trans_stats'], 'trans',
            self.abs_err_stats_fn)
        res_writer.update_and_save_stats(
            self.abs_errors['abs_e_rot_stats'], 'rot',
            self.abs_err_stats_fn)
        res_writer.update_and_save_stats(
            self.abs_errors['abs_e_scale_stats'], 'scale',
            self.abs_err_stats_fn)

        self.rel_error_stats_fns = []
        for dist in self.rel_errors:
            cur_err = self.rel_errors[dist]
            dist_str = "{:3.1f}".format(dist).replace('.', '_')
            dist_fn = os.path.join(
                self.saved_results_dir,
                Trajectory.rel_error_prefix+dist_str + 
                self.suffix_str + '.yaml')
            for et, label in zip(kRelMetrics, kRelMetricLables):
                res_writer.update_and_save_stats(
                    cur_err[et+'_stats'], label, dist_fn)

            self.rel_error_stats_fns.append(dist_fn)

    def compute_relative_error_at_subtraj_len(self, subtraj_len,
                                              max_dist_diff=-1):
        if max_dist_diff < 0:
            max_dist_diff = 0.2 * subtraj_len

        if self.rel_errors and (subtraj_len in self.rel_errors):
            print("Relative error at sub-trajectory length {0} is already "
                  "computed or loaded from cache.".format(subtraj_len))
        else:
            print("Computing relative error at sub-trajectory "
                  "length {0}".format(subtraj_len))
            Tcm = np.identity(4)
            _, e_trans, e_trans_perc, e_yaw, e_gravity, e_rot, e_rot_deg_per_m =\
                traj_err.compute_relative_error(
                    self.p_es, self.q_es, self.p_gt, self.q_gt, Tcm,
                    subtraj_len, max_dist_diff, self.accum_distances,
                    self.scale)
            dist_rel_err = {'rel_trans': e_trans,
                            'rel_trans_stats':
                            res_writer.compute_statistics(e_trans),
                            'rel_trans_perc': e_trans_perc,
                            'rel_trans_perc_stats':
                            res_writer.compute_statistics(e_trans_perc),
                            'rel_rot': e_rot,
                            'rel_rot_stats':
                            res_writer.compute_statistics(e_rot),
                            'rel_yaw': e_yaw,
                            'rel_yaw_stats':
                            res_writer.compute_statistics(e_yaw),
                            'rel_gravity': e_gravity,
                            'rel_gravity_stats':
                            res_writer.compute_statistics(e_gravity),
                            'rel_rot_deg_per_m': e_rot_deg_per_m,
                            'rel_rot_deg_per_m_stats':
                            res_writer.compute_statistics(e_rot_deg_per_m)}
            self.rel_errors[subtraj_len] = dist_rel_err
        return True

    def compute_relative_errors(self, subtraj_lengths=[]):
        suc = True
        if subtraj_lengths:
            for l in subtraj_lengths:
                suc = suc and self.compute_relative_error_at_subtraj_len(l)
        else:
            print(Fore.RED+"Computing the relative errors based on preset"
                  " subtrajectory lengths...")
            for l in self.preset_boxplot_distances:
                suc = suc and self.compute_relative_error_at_subtraj_len(l)
        self.success = suc
        print(Fore.GREEN+"...done.")

    def get_relative_errors_and_distances(
            self, error_types=['rel_trans', 'rel_trans_perc', 'rel_yaw']):
        rel_errors = {}
        for err_i in error_types:
            assert err_i in kRelMetrics
            rel_errors[err_i] = [[self.rel_errors[d][err_i]
                                 for d in self.preset_boxplot_distances]]
        return rel_errors, self.preset_boxplot_distances
