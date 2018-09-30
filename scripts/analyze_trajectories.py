#!/usr/bin/env python2

import os
import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

import add_path
from trajectory import Trajectory
import plot_utils as pu
import results_writer as res_writer

rc('font', **{'family': 'serif', 'serif': ['Cardo']})
rc('text', usetex=True)

FORMAT = '.pdf'

ALGORITHM_CONFIGS = ['vio_mono', 'vio_stereo']

# These are the labels that will be displayed for items in ALGORITHM_CONFIGS
PLOT_LABELS = {'vio_mono': 'vio mono',
               'vio_stereo': 'vio stereo'}

# assgin colors to different configurations
# make use you have more colors in the pallete!
pallete = ['b', 'g', 'r', 'c', 'k', 'y', 'm']
assert len(pallete) > len(
    ALGORITHM_CONFIGS), "Not enough colors for all configurations"
COLORS = {}
for i in range(len(ALGORITHM_CONFIGS)):
    COLORS[ALGORITHM_CONFIGS[i]] = pallete[i]

# DATASETS = ['MH_01', 'MH_02', 'MH_03', 'MH_04', 'MH_05', 'V1_01',
            # 'V1_02', 'V1_03', 'V2_01', 'V2_02', 'V2_03']
DATASETS = ['MH_01', 'MH_03', 'MH_05', 'V2_01', 'V2_02', 'V2_03']

# The maximum lenght will be used to calculate the relative error.
# otherwise it is calculated from the groundtruth
MAX_TRAJ_LENGTHS = {'MH_01': 80.6,
                    'MH_02': 73.4,
                    'MH_03': 130.9,
                    'MH_04': 91.7,
                    'MH_05': 97.5,
                    'V1_01': 58.5,
                    'V1_02': 75.8,
                    'V1_03': 78.9,
                    'V2_01': 36.4,
                    'V2_02': 83.2,
                    'V2_03': 86.1}
# boxplot distances that will be used for all datasets for overall errors
OVERALL_BOXPLOT_DISTANCES = [7.0, 14.0, 21.0, 28.0, 35.0]


def compute_odometry_error_per_dataset(dataset_trajectories_dict,
                                       dataset_names):
    dataset_rel_err = []
    print("\n>>> Calcuating relative error (KITTI style)...")
    for dataset_idx, dataset_nm in enumerate(dataset_names):
        print("> Processing {0} for all configurations...".format(
            dataset_nm))
        dataset_trajs = dataset_trajectories_dict[dataset_idx]

        cur_res = {'trans_err': {}, 'trans_err_perc': {},
                   'ang_yaw_err': {}, 'subtraj_len': []}

        traj_length = 0
        if(dataset_nm in MAX_TRAJ_LENGTHS):
            traj_length = MAX_TRAJ_LENGTHS[dataset_nm]
        else:
            for x in dataset_trajs:
                traj_length = max([x.traj_length, traj_length])
        print('Max trajectory length of all configurations: '+str(traj_length))
        distances = [np.floor(x*traj_length)
                     for x in [0.1, 0.2, 0.3, 0.4, 0.5]]
        cur_res['subtraj_len'] = distances
        print("Using distances {0} for relative error.".format(distances))

        for traj in dataset_trajs:
            cur_res['trans_err'][traj.alg] = []
            cur_res['trans_err_perc'][traj.alg] = []
            cur_res['ang_yaw_err'][traj.alg] = []

            print('Platform: '+traj.platform+' Alg: ' +
                  traj.alg+' Dataset: '+traj.dataset_short_name)

            for dist in distances:
                traj.compute_relative_error_at_subtraj_len(dist)

                cur_res['trans_err'][traj.alg].append(
                    traj.rel_errors[dist]['rel_trans'])
                cur_res['trans_err_perc'][traj.alg].append(
                    traj.rel_errors[dist]['rel_trans_perc'])
                cur_res['ang_yaw_err'][traj.alg].append(
                    traj.rel_errors[dist]['rel_yaw'])

        dataset_rel_err.append(cur_res)
        print("< Finish processing {0} for all configurations.".format(
            dataset_nm))

    print("<<< ...finish computing relative error.\n")

    return dataset_rel_err


def plot_odometry_error_per_dataset(dataset_rel_err, dataset_names, out_dir):
    for dataset_idx, dataset_nm in enumerate(dataset_names):
        print("Plotting {0}...".format(dataset_nm))
        rel_err = dataset_rel_err[dataset_idx]
        distances = rel_err['subtraj_len']

        config_labels = []
        config_colors = []
        for v in rel_err['trans_err'].keys():
            config_labels.append(PLOT_LABELS[v])
            config_colors.append(COLORS[v])

        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]',
            ylabel='Translation error [m]')
        pu.boxplot_compare(ax, distances, rel_err['trans_err'].values(),
                           config_labels, config_colors)
        fig.tight_layout()
        fig.savefig(output_dir+'/'+dataset_nm +
                    '_translation_error'+FORMAT, bbox_inches="tight")
        plt.close(fig)

        # relative error
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]',
            ylabel='Translation error [\%]')
        pu.boxplot_compare(ax, distances, rel_err['trans_err_perc'].values(),
                           config_labels, config_colors)
        fig.tight_layout()
        fig.savefig(output_dir+'/'+dataset_nm +
                    '_translation_error_percentage'+FORMAT,
                    bbox_inches="tight")
        plt.close(fig)

        # yaw orientation error
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]', ylabel='Yaw error [deg]')
        pu.boxplot_compare(ax, distances, rel_err['ang_yaw_err'].values(),
                           config_labels, config_colors)
        fig.tight_layout()
        fig.savefig(output_dir+'/'+dataset_nm +
                    '_yaw_error'+FORMAT, bbox_inches="tight")
        plt.close(fig)


def plot_trajectories(dataset_trajectories_dict, dataset_names, output_dir):
    for dataset_idx, dataset_nm in enumerate(dataset_names):
        dataset_trajs = dataset_trajectories_dict[dataset_idx]
        p_es_0 = {}
        p_gt_0 = dataset_trajs[0].p_gt
        for traj in dataset_trajs:
            p_es_0[traj.alg] = traj.p_es_aligned

        print("Plotting {0}...".format(dataset_nm))

        # plot trajectory
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect='equal',
                             xlabel='x [m]', ylabel='y [m]')
        for alg in p_es_0:
            pu.plot_trajectory_top(ax, p_es_0[alg], COLORS[alg],
                                   PLOT_LABELS[alg])
        pu.plot_trajectory_top(ax, p_gt_0, 'm', 'Groundtruth')
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        fig.tight_layout()
        fig.savefig(output_dir+'/' + dataset_nm +
                    '_trajectory_top'+FORMAT, bbox_inches="tight")
        plt.close(fig)

        # plot trajectory side
        fig = plt.figure(figsize=(6, 2.2))
        ax = fig.add_subplot(111, aspect='equal',
                             xlabel='x [m]', ylabel='z [m]')
        for alg in p_es_0:
            pu.plot_trajectory_side(ax, p_es_0[alg], COLORS[alg], 
                                    PLOT_LABELS[alg])
        pu.plot_trajectory_side(ax, p_gt_0, 'm', 'Groundtruth')
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        fig.tight_layout()
        fig.savefig(output_dir+'/'+dataset_nm +
                    '_trajectory_side'+FORMAT, bbox_inches="tight")
        plt.close(fig)


def compute_overall_odometry_errors(config_traj_list,
                                    overall_boxplot_distances):
    print("\n>>> Calculating overall relative error on all datasets...")
    # Compute overall errors and plot
    total_trans_errors_rel = {}
    total_trans_errors = {}
    total_ang_yaw_errors = {}

    n_dist = len(overall_boxplot_distances)
    for config_trajectories in config_traj_list:
        config = config_trajectories[0].alg
        print('> Processing config {0}...'.format(config))
        total_trans_errors[config] = [[] for d in range(n_dist)]
        total_trans_errors_rel[config] = [[] for d in range(n_dist)]
        total_ang_yaw_errors[config] = [[] for d in range(n_dist)]
        for traj in config_trajectories:
            print("> {0}".format(traj.uid))
            for dist_idx, dist in enumerate(overall_boxplot_distances):
                traj.compute_relative_error_at_subtraj_len(dist)
                total_trans_errors[traj.alg][dist_idx].extend(
                    traj.rel_errors[dist]['rel_trans'])
                total_trans_errors_rel[traj.alg][dist_idx].extend(
                    traj.rel_errors[dist]['rel_trans_perc'])
                total_ang_yaw_errors[traj.alg][dist_idx].extend(
                    traj.rel_errors[dist]['rel_yaw'])
        print('< ... analysis for config {0} done'.format(config))
    print("<<< ... calculating oveall relative error done.\n")

    overall_err = {}
    overall_err['trans_err'] = total_trans_errors
    overall_err['trans_err_perc'] = total_trans_errors_rel
    overall_err['yaw_err'] = total_ang_yaw_errors
    overall_err['distances'] = overall_boxplot_distances

    float_fmt = "{:3.2f}"
    overall_err_tables = {}
    overall_err_tables['trans_err'] = {}
    overall_err_tables['trans_err']['values'] = []
    overall_err_tables['trans_err_perc'] = {}
    overall_err_tables['trans_err_perc']['values'] = []
    overall_err_tables['yaw_err'] = {}
    overall_err_tables['yaw_err']['values'] = []

    rows = []
    for dist_idx, dist in enumerate(overall_boxplot_distances):
        dist_str = float_fmt.format(dist)+'m'
        rows.append(dist_str)

        rel_trans_err_i = []
        rel_trans_err_perc_i = []
        rel_yaw_err_i = []
        for config_trajs in config_traj_list:
            alg = config_trajs[0].alg
            mean_trans_e = np.mean(overall_err['trans_err'][alg][dist_idx])
            rel_trans_err_i.append(float_fmt.format(mean_trans_e))
            mean_trans_perc_e =\
                np.mean(overall_err['trans_err_perc'][alg][dist_idx])
            rel_trans_err_perc_i.append(float_fmt.format(mean_trans_perc_e))
            mean_yaw_e = np.mean(overall_err['yaw_err'][alg][dist_idx])
            rel_yaw_err_i.append(float_fmt.format(mean_yaw_e))

        overall_err_tables['trans_err']['values'].append(rel_trans_err_i)
        overall_err_tables['trans_err_perc']['values'].append(
            rel_trans_err_perc_i)
        overall_err_tables['yaw_err']['values'].append(rel_yaw_err_i)
    cols = []
    for config_trajs in config_traj_list:
        alg = config_trajs[0].alg
        cols.append(alg)
    overall_err_tables['trans_err']['cols'] = cols
    overall_err_tables['trans_err']['rows'] = rows
    overall_err_tables['trans_err_perc']['cols'] = cols
    overall_err_tables['trans_err_perc']['rows'] = rows
    overall_err_tables['yaw_err']['cols'] = cols
    overall_err_tables['yaw_err']['rows'] = rows

    return overall_err, overall_err_tables


def write_overall_odometry_errors_table(overall_err_tables, output_dir):
    print("Writing to overall error to text files...")
    res_writer.write_tex_table(overall_err_tables['trans_err']['values'],
                               overall_err_tables['trans_err']['rows'],
                               overall_err_tables['trans_err']['cols'],
                               os.path.join(output_dir,
                                            'overall_rel_trans_err.txt'))
    res_writer.write_tex_table(overall_err_tables['trans_err_perc']['values'],
                               overall_err_tables['trans_err_perc']['rows'],
                               overall_err_tables['trans_err_perc']['cols'],
                               os.path.join(output_dir,
                                            'overall_rel_trans_err_perc.txt'))
    res_writer.write_tex_table(overall_err_tables['yaw_err']['values'],
                               overall_err_tables['yaw_err']['rows'],
                               overall_err_tables['yaw_err']['cols'],
                               os.path.join(output_dir,
                                            'overall_rel_yaw_err.txt'))


def plot_overall_odometry_errors(overall_err, output_dir):
    print("Plotting overall error...")
    n_config = len(overall_err['trans_err'])
    assert n_config == len(overall_err['trans_err_perc'])
    assert n_config == len(overall_err['yaw_err'])

    config_labels = []
    config_colors = []
    for v in overall_err['trans_err'].keys():
        config_labels.append(PLOT_LABELS[v])
        config_colors.append(COLORS[v])

    # absolute error
    fig = plt.figure(figsize=(6, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance traveled [m]', ylabel='Translation error [m]')
    pu.boxplot_compare(ax, overall_err['distances'],
                       overall_err['trans_err'].values(),
                       config_labels, config_colors)
    fig.tight_layout()
    fig.savefig(output_dir+'/overall_rel_translation_error' +
                FORMAT, bbox_inches="tight")
    plt.close(fig)

    # relative error
    fig = plt.figure(figsize=(6, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance traveled [m]', ylabel='Translation error [\%]')
    pu.boxplot_compare(ax, overall_err['distances'],
                       overall_err['trans_err_perc'].values(),
                       config_labels, config_colors)
    fig.tight_layout()
    fig.savefig(output_dir+'/overall_rel_translation_error_percentage' +
                FORMAT, bbox_inches="tight")
    plt.close(fig)

    # yaw orientation error
    fig = plt.figure(figsize=(6, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance traveled [m]', ylabel='Yaw error [deg]')
    pu.boxplot_compare(ax, overall_err['distances'],
                       overall_err['yaw_err'].values(),
                       config_labels, config_colors)
    fig.tight_layout()
    fig.savefig(output_dir+'/overall_rel_yaw_error'+FORMAT,
                bbox_inches="tight")
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''Analyze trajectories''')

    default_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../results')
    parser.add_argument(
        '--output_dir',
        help="Folder to output plots and data",
        default=default_path)
    parser.add_argument(
        '--results_dir', help='base folder with the results to analyze',
        default=default_path)

    parser.add_argument(
        '--platform', help='HW platform: [laptop, nuc, odroid, up]',
        default='laptop')
    parser.add_argument(
        '--alg',
        help='Algorithm configurations',
        default='all')
    parser.add_argument(
        '--dataset',
        help='[MH_01,...,MH_05,V1_01,...,V1_03,V2_01,...,V2_03, all]',
        default='all')

    parser.add_argument(
        '--odometry_error_per_dataset',
        help="Analyze odometry error for individual dataset. "
        "The same subtrajectory length will be used for the same dataset "
        "and different algorithms",
        action='store_true')
    parser.add_argument(
        '--overall_odometry_error',
        help="accumulate and analyze the odometry error over all datasets. "
        "Fixed distances will be used for all datasets and algorithms.",
        action='store_true')
    parser.add_argument(
        '--rmse_table', help='Output rms erros into latex tables',
        action='store_true')

    parser.add_argument('--recalculate_errors',
                        help='Deletes cached errors', action='store_true')
    parser.add_argument('--plot_trajectories',
                        help='Plot the trajectories', action='store_true')
    parser.add_argument('--png',
                        help='Save plots as png instead of pdf',
                        action='store_true')

    args = parser.parse_args()

    print("Will analyze results from {0} and output will be "
          "in {1}".format(args.results_dir, args.output_dir))

    output_dir = args.output_dir

    alg = [args.alg]
    if args.alg == 'all' or args.alg is None:
        alg = ALGORITHM_CONFIGS
    else:
        ALGORITHM_CONFIGS = alg
    print('Will process configurations: {0}'.format(alg))

    datasets = [args.dataset]
    if args.dataset is 'all':
        datasets = DATASETS
    else:
        DATASETS = [datasets]
    print("Will process datasets: {0}".format(datasets))

    if args.png:
        FORMAT = '.png'

    print("#####################################")
    print(">>> Start loading and preprocessing all trajectories...")
    print("#####################################")
    # organize by configuration
    config_trajectories_list = []
    for config_i in alg:
        cur_trajectories_i = []
        for d in datasets:
            print("--- Processing {0}-{1}... ---".format(config_i, d))
            trial_name = args.platform + '_' + config_i + '_' + d
            trace_dir = os.path.join(args.results_dir,
                                     args.platform, config_i, trial_name)
            assert os.path.exists(trace_dir), "No corresponding trace dir"
            if args.recalculate_errors:
                Trajectory.remove_cached_error(trace_dir)

            cur_traj = Trajectory(trace_dir, platform=args.platform,
                                  alg_name=config_i, dataset_name=d)

            cur_trajectories_i.append(cur_traj)
        config_trajectories_list.append(cur_trajectories_i)

    # organize by dataset name
    dataset_trajectories_list = []
    for ds_idx, dataset_nm in enumerate(datasets):
        dataset_trajs = [v[ds_idx] for v in config_trajectories_list]
        dataset_trajectories_list.append(dataset_trajs)

    print("#####################################")
    print(">>> Start computing error metrics....")
    print("#####################################")
    print("\n>>> Computing absolute trajectory errors...")
    rmse_table = {}
    rmse_table['values'] = []
    for config_trajs in config_trajectories_list:
        cur_trans_rmse = []
        for traj in config_trajs:
            print("> Processing {0}".format(traj.uid))
            traj.compute_absolute_error()
            cur_trans_rmse.append("{:3.2f}".format(
                traj.abs_errors['abs_e_trans_stats']['rmse']))
        rmse_table['values'].append(cur_trans_rmse)
    rmse_table['rows'] = alg
    rmse_table['cols'] = datasets
    print("<<< ...computing absolute trajectory errors done.\n")

    dataset_rel_err = {}
    if args.odometry_error_per_dataset:
        dataset_rel_err = compute_odometry_error_per_dataset(
            dataset_trajectories_list, datasets)

    overall_err = {}
    overall_err_tables = {}
    if args.overall_odometry_error:
        overall_err, overall_err_tables = compute_overall_odometry_errors(
            config_trajectories_list, OVERALL_BOXPLOT_DISTANCES)

    print("#####################################")
    print(">>> Save computed errors....")
    print("#####################################")
    for config_trajs in config_trajectories_list:
        for traj in config_trajs:
            print('> Saving {0}'.format(traj.uid))
            traj.cache_current_error()
            traj.write_errors_to_yaml()

    print("#####################################")
    print(">>> Start plotting and writing results....")
    print("#####################################")
    if args.plot_trajectories:
        print('\n--- Plotting trajectory top and side view ... ---')
        plot_trajectories(dataset_trajectories_list, datasets, output_dir)
    if args.odometry_error_per_dataset:
        print('\n--- Generating relative (KITTI style) error plots... ---')
        plot_odometry_error_per_dataset(dataset_rel_err, datasets, output_dir)
    if args.overall_odometry_error:
        print('\n--- Plotting overall error ... ---')
        plot_overall_odometry_errors(overall_err, output_dir)
        write_overall_odometry_errors_table(overall_err_tables, output_dir)
    if args.rmse_table:
        print('\n--- Generating RMSE tables... ---')
        res_writer.write_tex_table(
            rmse_table['values'], rmse_table['rows'], rmse_table['cols'],
            os.path.join(output_dir, args.platform + '_translation_rmse.txt'))

    print("#####################################")
    print("<<< Finished.")
    print("#####################################")
