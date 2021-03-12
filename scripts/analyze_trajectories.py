#!/usr/bin/env python2

import os
import argparse
from ruamel.yaml import YAML
import shutil
import json
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
from colorama import init, Fore

import add_path
from trajectory import Trajectory
import plot_utils as pu
import results_writer as res_writer
from analyze_trajectory_single import analyze_multiple_trials
from fn_constants import kNsToEstFnMapping, kNsToMatchFnMapping, kFnExt

init(autoreset=True)

rc('font', **{'family': 'serif', 'serif': ['Cardo']})
rc('text', usetex=True)

FORMAT = '.pdf'

def spec(N):                                             
    t = np.linspace(-510, 510, N)                                              
    return np.round(np.clip(np.stack([-t, 510-np.abs(t), t], axis=1), 0, 255)).astype("float32")/255

PALLETE = spec(20)


def collect_odometry_error_per_dataset(dataset_multierror_list,
                                       dataset_names):
    dataset_rel_err = []
    print("\n>>> Collecting relative error (KITTI style)...")
    for dataset_idx, dataset_nm in enumerate(dataset_names):
        print("> Processing {0} for all configurations...".format(
            dataset_nm))
        dataset_mt_error = dataset_multierror_list[dataset_idx]
        print("> Found {0} configurations.".format(len(dataset_mt_error)))
        for d in dataset_mt_error:
            print('  - {0}: {1}'.format(d.alg, d.uid))

        cur_res = {'trans_err': {}, 'trans_err_perc': {},
                   'ang_yaw_err': {}, 'rot_deg_per_m': {},
                   'subtraj_len': []}

        cur_res['subtraj_len'] = dataset_mt_error[0].rel_distances
        for d in dataset_mt_error:
            assert cur_res['subtraj_len'] == d.rel_distances,\
                "inconsistent boxplot distances"
        print("Using distances {0} for relative error.".format(
            cur_res['subtraj_len']))

        print("Processing each configurations...")
        for mt_error in dataset_mt_error:
            cur_alg = mt_error.alg
            print('  - {0}'.format(cur_alg))
            cur_res['trans_err'][cur_alg] = []
            cur_res['trans_err_perc'][cur_alg] = []
            cur_res['ang_yaw_err'][cur_alg] = []
            cur_res['rot_deg_per_m'][cur_alg] = []

            for dist in cur_res['subtraj_len']:
                cur_res['trans_err'][cur_alg].append(
                    mt_error.rel_errors[dist]['rel_trans'])
                cur_res['trans_err_perc'][cur_alg].append(
                    mt_error.rel_errors[dist]['rel_trans_perc'])
                cur_res['ang_yaw_err'][cur_alg].append(
                    mt_error.rel_errors[dist]['rel_yaw'])
                cur_res['rot_deg_per_m'][cur_alg].append(
                    mt_error.rel_errors[dist]['rel_rot_deg_per_m'])

        dataset_rel_err.append(cur_res)
        print("< Finish processing {0} for all configurations.".format(
            dataset_nm))

    print("<<< ...finish collecting relative error.\n")

    return dataset_rel_err


def plot_odometry_error_per_dataset(dataset_rel_err, dataset_names, algorithm_names,
                                    datasets_outdir, plot_settings):
    for dataset_idx, dataset_nm in enumerate(dataset_names):
        output_dir = datasets_outdir[dataset_nm]
        print("Plotting {0}...".format(dataset_nm))
        rel_err = dataset_rel_err[dataset_idx]
        assert sorted(algorithm_names) == sorted(list(rel_err['trans_err'].keys()))
        distances = rel_err['subtraj_len']

        config_labels = []
        config_colors = []
        for v in algorithm_names:
            config_labels.append(plot_settings['algo_labels'][v])
            config_colors.append(plot_settings['algo_colors'][v])

        fig = plt.figure(figsize=(12, 3))
        ax = fig.add_subplot(
            121, xlabel='Distance traveled (m)',
            ylabel='Translation error (\%)')
        pu.boxplot_compare(ax, distances, [rel_err['trans_err_perc'][v] for v in algorithm_names],
                           config_labels, config_colors, legend=False)
        ax = fig.add_subplot(
            122, xlabel='Distance traveled (m)', ylabel='Rotation error (deg / m)')
        pu.boxplot_compare(ax, distances, [rel_err['rot_deg_per_m'][v] for v in algorithm_names],
                           config_labels, config_colors, legend=True)
        fig.tight_layout()
        fig.savefig(output_dir+'/'+dataset_nm +
                    '_trans_rot_error'+FORMAT, bbox_inches="tight", dpi=args.dpi)
        plt.close(fig)


def collect_rmse_per_dataset(config_multierror_list,
                             algorithms):
    algorithm_rmse = {'trans_err': {}, 'rot_err': {}}
    print("\n>>> Collecting RMSE per dataset...")
    for idx, alg_i in enumerate(algorithms):
        config_mt_error = config_multierror_list[idx]
        algorithm_rmse['trans_err'][alg_i] = []
        algorithm_rmse['rot_err'][alg_i] = []
        for mt_error in config_mt_error:
            algorithm_rmse['trans_err'][alg_i].append(
                mt_error.abs_errors['rmse_trans'])
            algorithm_rmse['rot_err'][alg_i].append(
                mt_error.abs_errors['rmse_rot'])

    return algorithm_rmse


def plot_rmse_per_dataset(algorithm_rmse, dataset_names, algorithm_names,
                          output_dir, plot_settings):
    config_labels = []
    config_colors = []
    assert sorted(algorithm_names) == sorted(list(algorithm_rmse['trans_err'].keys()))
    for v in algorithm_names:
        config_labels.append(plot_settings['algo_labels'][v])
        config_colors.append(plot_settings['algo_colors'][v])
    assert len(datasets_labels) == len(datasets)
    labels = [plot_settings['datasets_labels'][v] for v in dataset_names]

    # check
    n_data = len(datasets)
    for v in algorithm_names:
        assert len(algorithm_rmse['trans_err'][v]) == n_data
        assert len(algorithm_rmse['rot_err'][v]) == n_data

    fig = plt.figure(figsize=(6, 3))
    ax = fig.add_subplot(
        111, xlabel='Datasets',
        ylabel='Translation RMSE (m)')
    pu.boxplot_compare(ax, labels,
                       [algorithm_rmse['trans_err'][v] for v in algorithm_names],
                       config_labels, config_colors)
    fig.tight_layout()
    fig.savefig(output_dir+'/' +
                'all_translation_rmse'+FORMAT, bbox_inches="tight", dpi=args.dpi)
    plt.close(fig)

    fig = plt.figure(figsize=(6, 3))
    ax = fig.add_subplot(
        111, xlabel='Datasets',
        ylabel='Rotation RMSE (deg)')
    pu.boxplot_compare(ax, labels, [algorithm_rmse['rot_err'][v] for v in algorithm_names],
                       config_labels, config_colors)
    fig.tight_layout()
    fig.savefig(output_dir+'/' +
                'all_rotation_rmse'+FORMAT, bbox_inches="tight", dpi=args.dpi)
    plt.close(fig)


def plot_trajectories(dataset_trajectories_list, dataset_names, algorithm_names,
                      datasets_out_dir, plot_settings, plot_idx=0, plot_side=True,
                      plot_aligned=True, plot_traj_per_alg=True):
    for dataset_idx, dataset_nm in enumerate(dataset_names):
        output_dir = datasets_out_dir[dataset_nm]
        dataset_trajs = dataset_trajectories_list[dataset_idx]
        p_es_0 = {}
        p_gt_raw = (dataset_trajs[0])[plot_idx].p_gt_raw
        p_gt_0 = {}
        for traj_list in dataset_trajs:
            p_es_0[traj_list[plot_idx].alg] = traj_list[plot_idx].p_es_aligned
            p_gt_0[traj_list[plot_idx].alg] = traj_list[plot_idx].p_gt
        print("Collected trajectories to plot: {0}".format(algorithm_names))
        assert sorted(algorithm_names) == sorted(list(p_es_0.keys()))

        print("Plotting {0}...".format(dataset_nm))

        # plot trajectory
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect='equal',
                             xlabel='x [m]', ylabel='y [m]')
        if dataset_nm in plot_settings['datasets_titles']:
            ax.set_title(plot_settings['datasets_titles'][dataset_nm])
        for alg in algorithm_names:
            if plot_traj_per_alg:
                fig_i = plt.figure(figsize=(6, 5.5))
                ax_i = fig_i.add_subplot(111, aspect='equal',
                                         xlabel='x [m]', ylabel='y [m]')
                pu.plot_trajectory_top(ax_i, p_es_0[alg], 'b',
                                       'Estimate ' + plot_settings['algo_labels'][alg], 0.5)
                pu.plot_trajectory_top(ax_i, p_gt_0[alg], 'm', 'Groundtruth')
                if plot_aligned:
                    pu.plot_aligned_top(ax_i, p_es_0[alg], p_gt_0[alg], -1)
                plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
                fig_i.tight_layout()
                fig_i.savefig(output_dir+'/' + dataset_nm + '_trajectory_top_' +
                              plot_settings['algo_labels'][alg] + FORMAT,
                              bbox_inches="tight", dpi=args.dpi)
                plt.close(fig_i)
            pu.plot_trajectory_top(ax, p_es_0[alg],
                                   plot_settings['algo_colors'][alg],
                                   plot_settings['algo_labels'][alg])
        plt.sca(ax)
        pu.plot_trajectory_top(ax, p_gt_raw, 'm', 'Groundtruth')
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        fig.tight_layout()
        fig.savefig(output_dir+'/' + dataset_nm +
                    '_trajectory_top'+FORMAT, bbox_inches="tight", dpi=args.dpi)
        plt.close(fig)

        # plot trajectory side
        if not plot_side:
            continue
        fig = plt.figure(figsize=(6, 2.2))
        ax = fig.add_subplot(111, aspect='equal',
                             xlabel='x [m]', ylabel='z [m]')
        if dataset_nm in plot_settings['datasets_titles']:
            ax.set_title(plot_settings['datasets_titles'][dataset_nm])
        for alg in algorithm_names:
            if plot_traj_per_alg:
                fig_i = plt.figure(figsize=(6, 5.5))
                ax_i = fig_i.add_subplot(111, aspect='equal',
                                         xlabel='x [m]', ylabel='y [m]')
                pu.plot_trajectory_side(ax_i, p_es_0[alg], 'b',
                                        'Estimate ' + plot_settings['algo_labels'][alg], 0.5)
                pu.plot_trajectory_side(ax_i, p_gt_0[alg], 'm', 'Groundtruth')
                plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
                fig_i.tight_layout()
                fig_i.savefig(output_dir+'/' + dataset_nm + '_trajectory_side_' +
                              plot_settings['algo_labels'][alg] + FORMAT,
                              bbox_inches="tight", dpi=args.dpi)
                plt.close(fig_i)
            pu.plot_trajectory_side(ax, p_es_0[alg],
                                    plot_settings['algo_colors'][alg],
                                    plot_settings['algo_labels'][alg])
        plt.sca(ax)
        pu.plot_trajectory_side(ax, p_gt_raw, 'm', 'Groundtruth')
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        fig.tight_layout()
        fig.savefig(output_dir+'/'+dataset_nm +
                    '_trajectory_side'+FORMAT, bbox_inches="tight", dpi=args.dpi)
        plt.close(fig)


def collect_odometry_error_per_algorithm(config_multierror_list, algorithms, distances,
                                         rel_keys=['rel_trans_perc', 'rel_rot_deg_per_m']):
    odometry_error_collection = {}
    print("Distances: {}".format(distances))
    for et in rel_keys:
        print(">> Error type {}".format(et))
        type_e_dict = {}
        for idx, alg in enumerate(algorithms):
            alg_dist_err = {}
            for mt_error in config_multierror_list[idx]:
                cal_distances = distances[:]
                if not cal_distances:
                    cal_distances = mt_error.rel_errors.keys()
                for dist in cal_distances:
                    errors = mt_error.rel_errors[dist]
                    if dist not in alg_dist_err:
                        alg_dist_err[dist] = errors[et].tolist()
                    else:
                        alg_dist_err[dist].extend(errors[et].tolist())
            type_e_dict[alg] = alg_dist_err
        odometry_error_collection[et] = type_e_dict

    return odometry_error_collection


def plot_overall_odometry_errors(odo_err_col, algorithm_names, rel_e_distances,
                                 plot_settings, output_dir):
    for et in odo_err_col:
        if et == 'rel_trans_perc':
            ylabel = 'Translation error (\%)'
        elif et == 'rel_rot_deg_per_m':
            ylabel = 'Rotation error (deg / m)'
        else:
            assert False
        cur_err = odo_err_col[et]

        assert sorted(algorithm_names) == sorted(list(cur_err.keys()))
        colors = []
        labels = []
        for alg in algorithm_names:
            colors.append(plot_settings['algo_colors'][alg])
            labels.append(plot_settings['algo_labels'][alg])

        distances = sorted(rel_e_distances)
        errors = []
        for alg in algorithm_names:
            errors.append([cur_err[alg][d] for d in distances])

        fig = plt.figure(figsize=(12, 3))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]', ylabel=ylabel)
        pu.boxplot_compare(ax, distances, errors,
                           labels, colors, legend=True)
        fig.tight_layout()
        fig.savefig(output_dir+'/' + 'overall_{}'.format(et)+FORMAT,
                    bbox_inches="tight", dpi=args.dpi)
        plt.close(fig)


def parse_config_file(config_fn, sort_names):
    yaml = YAML()
    with open(config_fn) as f:
        d = yaml.load(f)
    datasets = d['Datasets'].keys()
    if sort_names:
        datasets = sorted(datasets)
    datasets_labels = {}
    datasets_titles = {}
    for v in datasets:
        datasets_labels[v] = d['Datasets'][v]['label']
        if 'title' in d['Datasets'][v]:
            datasets_titles[v] = d['Datasets'][v]['title']

    algorithms = d['Algorithms'].keys()
    if sort_names:
        algorithms = sorted(algorithms)
    alg_labels = {}
    alg_fn = {}
    for v in algorithms:
        alg_labels[v] = d['Algorithms'][v]['label']
        alg_fn[v] = d['Algorithms'][v]['fn']

    boxplot_distances = []
    if 'RelDistances' in d:
        boxplot_distances = d['RelDistances']
    boxplot_percentages = []
    if 'RelDistancePercentages' in d:
        boxplot_percentages = d['RelDistancePercentages']

    if boxplot_distances and boxplot_percentages:
        print(Fore.RED + "Found both both distances and percentages for boxplot distances")
        print(Fore.RED + "Will use the distances instead of percentages.")
        boxplot_percentages = []

    return datasets, datasets_labels, datasets_titles, algorithms, alg_labels, alg_fn,\
        boxplot_distances, boxplot_percentages


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''Analyze trajectories''')

    default_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../results')
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               '../analyze_trajectories_config')

    parser.add_argument('config', type=str,
                        help='yaml file specifying algorithms and datasets')
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
        '--mul_trials', type=int,
        help='number of trials, None for single run', default=None)
    parser.add_argument('--no_sort_names', action='store_false', dest='sort_names',
                        help='whether to sort dataset and algorithm names')

    # odometry error
    parser.add_argument(
        '--odometry_error_per_dataset',
        help="Analyze odometry error for individual dataset. "
        "The same subtrajectory length will be used for the same dataset "
        "and different algorithms",
        action='store_true')
    parser.add_argument(
        '--overall_odometry_error',
        help="Collect the odometry error from all datasets and calculate statistics.",
        dest='overall_odometry_error',
        action='store_true')

    # RMSE (ATE)
    parser.add_argument(
        '--rmse_table', help='Output rms erros into latex tables',
        action='store_true')
    parser.add_argument(
        '--rmse_table_median_only', action='store_true', dest='rmse_median_only')
    parser.add_argument('--rmse_boxplot',
                        help='Plot the trajectories', action='store_true')

    parser.add_argument('--write_time_statistics', help='write time statistics',
                        action='store_true')

    # plot trajectories
    parser.add_argument('--plot_trajectories',
                        help='Plot the trajectories', action='store_true')
    parser.add_argument('--no_plot_side', action='store_false', dest='plot_side')
    parser.add_argument('--no_plot_aligned', action='store_false', dest='plot_aligned')
    parser.add_argument('--no_plot_traj_per_alg', action='store_false',
                        dest='plot_traj_per_alg')

    parser.add_argument('--recalculate_errors',
                        help='Deletes cached errors', action='store_true')
    parser.add_argument('--png',
                        help='Save plots as png instead of pdf',
                        action='store_true')
    parser.add_argument('--dpi', type=int, default=300)
    parser.set_defaults(odometry_error_per_dataset=False, overall_odometry_error=False,
                        rmse_table=False,
                        plot_trajectories=False, rmse_boxplot=False,
                        recalculate_errors=False, png=False, time_statistics=False,
                        sort_names=True, plot_side=True, plot_aligned=True,
                        plot_traj_per_alg=True, rmse_median_only=False)
    args = parser.parse_args()
    print("Arguments:\n{}".format(
        '\n'.join(['- {}: {}'.format(k, v)
                   for k, v in args.__dict__.items()])))

    print("Will analyze results from {0} and output will be "
          "in {1}".format(args.results_dir, args.output_dir))
    output_dir = args.output_dir

    config_fn = os.path.join(config_path, args.config)

    print("Parsing evaluation configuration {0}...".format(config_fn))

    datasets, datasets_labels, datasets_titles,\
        algorithms, algo_labels, algo_fn, rel_e_distances, rel_e_perc = \
        parse_config_file(config_fn, args.sort_names)
    shutil.copy2(config_fn, output_dir)
    datasets_res_dir = {}
    for d in datasets:
        cur_res_dir = os.path.join(output_dir, '{}_{}_results'.format(args.platform, d))
        if os.path.exists(cur_res_dir):
            shutil.rmtree(cur_res_dir)
        os.makedirs(cur_res_dir)
        datasets_res_dir[d] = cur_res_dir
    same_subtraj = True if rel_e_distances else False
    assert len(PALLETE) > len(algorithms),\
        "Not enough colors for all configurations"
    algo_colors = {}
    for i in range(len(algorithms)):
        algo_colors[algorithms[i]] = PALLETE[i]

    print(Fore.YELLOW+"=== Evaluation Configuration Summary ===")
    print(Fore.YELLOW+"Datasests to evaluate: ")
    for d in datasets:
        print(Fore.YELLOW+'- {0}: {1}'.format(d, datasets_labels[d]))
    print(Fore.YELLOW+"Algorithms to evaluate: ")
    for a in algorithms:
        print(Fore.YELLOW+'- {0}: {1}, {2}, {3}'.format(a, algo_labels[a],
                                                        algo_fn[a],
                                                        algo_colors[a]))
    plot_settings = {'datasets_labels': datasets_labels,
                     'datasets_titles': datasets_titles,
                     'algo_labels': algo_labels,
                     'algo_colors': algo_colors}

    if args.png:
        FORMAT = '.png'

    eval_uid = '_'.join(list(plot_settings['algo_labels'].values())) +\
        datetime.now().strftime("%Y%m%d%H%M")

    n_trials = 1
    if args.mul_trials:
        print(Fore.YELLOW +
              "We will ananlyze multiple trials #{0}".format(args.mul_trials))
        n_trials = args.mul_trials

    need_odometry_error = args.odometry_error_per_dataset or args.overall_odometry_error
    if need_odometry_error:
        print(Fore.YELLOW+"Will calculate odometry errors")

    print("#####################################")
    print(">>> Loading and calculating errors....")
    print("#####################################")
    # organize by configuration
    config_trajectories_list = []
    config_multierror_list = []
    dataset_boxdist_map = {}
    for d in datasets:
        dataset_boxdist_map[d] = rel_e_distances
    for config_i in algorithms:
        cur_trajectories_i = []
        cur_mulierror_i = []
        for d in datasets:
            print(Fore.RED + "--- Processing {0}-{1}... ---".format(
                config_i, d))
            exp_name = args.platform + '_' + config_i + '_' + d
            trace_dir = os.path.join(args.results_dir,
                                     args.platform, config_i, exp_name)
            assert os.path.exists(trace_dir),\
                "{0} not found.".format(trace_dir)
            traj_list, mt_error = analyze_multiple_trials(
                trace_dir, algo_fn[config_i], n_trials,
                recalculate_errors=args.recalculate_errors,
                preset_boxplot_distances=dataset_boxdist_map[d],
                preset_boxplot_percentages=rel_e_perc,
                compute_odometry_error=need_odometry_error)
            if not dataset_boxdist_map[d] and traj_list:
                print("Assign the boxplot distances for {0}...".format(d))
                dataset_boxdist_map[d] = traj_list[0].preset_boxplot_distances
            for traj in traj_list:
                traj.alg = config_i
                traj.dataset_short_name = d
            mt_error.saveErrors()
            mt_error.cache_current_error()
            mt_error.uid = '_'.join([args.platform, config_i,
                                     d, str(n_trials)])
            mt_error.alg = config_i
            mt_error.dataset = d
            cur_trajectories_i.append(traj_list)
            cur_mulierror_i.append(mt_error)
        config_trajectories_list.append(cur_trajectories_i)
        config_multierror_list.append(cur_mulierror_i)

    # organize by dataset name
    dataset_trajectories_list = []
    dataset_multierror_list = []
    for ds_idx, dataset_nm in enumerate(datasets):
        dataset_trajs = [v[ds_idx] for v in config_trajectories_list]
        dataset_trajectories_list.append(dataset_trajs)
        dataset_multierrors = [v[ds_idx] for v in config_multierror_list]
        dataset_multierror_list.append(dataset_multierrors)

    print("#####################################")
    print(">>> Analyze different error types...")
    print("#####################################")
    print(Fore.RED+">>> Processing absolute trajectory errors...")
    if args.rmse_table:
        rmse_table = {}
        rmse_table['values'] = []
        for config_mt_error in config_multierror_list:
            cur_trans_rmse = []
            for mt_error_d in config_mt_error:
                print("> Processing {0}".format(mt_error_d.uid))
                if args.rmse_median_only or n_trials == 1:
                    cur_trans_rmse.append(
                        "{:3.3f}".format(
                            mt_error_d.abs_errors['rmse_trans_stats']['median']))
                else:
                    cur_trans_rmse.append(
                        "{:3.3f}, {:3.3f} ({:3.3f} - {:3.3f})".format(
                            mt_error_d.abs_errors['rmse_trans_stats']['mean'],
                            mt_error_d.abs_errors['rmse_trans_stats']['median'],
                            mt_error_d.abs_errors['rmse_trans_stats']['min'],
                            mt_error_d.abs_errors['rmse_trans_stats']['max']))
            rmse_table['values'].append(cur_trans_rmse)
        rmse_table['rows'] = algorithms
        rmse_table['cols'] = datasets
        print('\n--- Generating RMSE tables... ---')
        res_writer.write_tex_table(
            rmse_table['values'], rmse_table['rows'], rmse_table['cols'],
            os.path.join(output_dir, args.platform + '_translation_rmse_' +
                         eval_uid+'.txt'))

    if args.rmse_boxplot and n_trials > 1:
        rmse_plot_alg = [v for v in algorithms]
        algorithm_rmse = collect_rmse_per_dataset(config_multierror_list,
                                                  rmse_plot_alg)
        print("--- Generate boxplot for RMSE ---")
        plot_rmse_per_dataset(algorithm_rmse, datasets, algorithms,
                              output_dir, plot_settings)
    print(Fore.GREEN+"<<< ...processing absolute trajectory errors done.")

    print(Fore.RED+">>> Collecting odometry errors per dataset...")
    if args.odometry_error_per_dataset:
        dataset_rel_err = {}
        dataset_rel_err = collect_odometry_error_per_dataset(
            dataset_multierror_list, datasets)
        print(Fore.MAGENTA +
              '--- Generating relative (KITTI style) error plots ---')
        plot_odometry_error_per_dataset(dataset_rel_err, datasets, algorithms,
                                        datasets_res_dir, plot_settings)
    print(Fore.GREEN+"<<< .... processing odometry errors done.\n")

    print(Fore.RED+">>> Collecting odometry errors per algorithms...")
    if args.overall_odometry_error:
        rel_err_names = ['rel_trans_perc', 'rel_rot_deg_per_m']
        rel_err_labels = ['Translation (\%)', 'Rotation (deg/meter)']
        all_odo_err = collect_odometry_error_per_algorithm(
            config_multierror_list, algorithms, rel_e_distances, rel_keys=rel_err_names)
        print(Fore.MAGENTA+'--- Plotting and writing overall odometry errors... ---')
        if same_subtraj:
            plot_overall_odometry_errors(
                all_odo_err, algorithms, rel_e_distances, plot_settings, output_dir)
        else:
            print("Skip plotting overall odometry error since datasets are evaluated at"
                  " different distances.")
        rel_err_table = {}
        rel_err_table['values'] = []
        for alg in algorithms:
            alg_aver_rel = []
            for et in rel_err_names:
                cur_errors = []
                for _, v in all_odo_err[et][alg].items():
                    cur_errors.extend(v)
                alg_aver_rel.append('{0:.3f}'.format(
                    res_writer.compute_statistics(cur_errors)['mean']))
            rel_err_table['values'].append(alg_aver_rel)
        rel_err_table['cols'] = rel_err_labels
        rel_err_table['rows'] = algorithms
        res_writer.write_tex_table(
            rel_err_table['values'], rel_err_table['rows'], rel_err_table['cols'],
            os.path.join(output_dir, args.platform + '_rel_err_' +
                         eval_uid+'.txt'))

    if args.plot_trajectories:
        print(Fore.MAGENTA+'--- Plotting trajectory top and side view ... ---')
        plot_trajectories(dataset_trajectories_list, datasets, algorithms,
                          datasets_res_dir, plot_settings, plot_side=args.plot_side,
                          plot_aligned=args.plot_aligned,
                          plot_traj_per_alg=args.plot_traj_per_alg)

    if args.write_time_statistics:
        dataset_alg_t_stats = []
        for didx, d in enumerate(datasets):
            cur_d_time_stats = {}
            for algidx, alg in enumerate(algorithms):
                cur_alg_t_stats = {'start': [], 'end': [], 'n_meas': []}
                for traj in dataset_trajectories_list[didx][algidx]:
                    cur_alg_t_stats['start'].append(traj.t_es[0])
                    cur_alg_t_stats['end'].append(traj.t_es[-1])
                    cur_alg_t_stats['n_meas'].append(traj.t_es.size)
                cur_d_time_stats[algo_labels[alg]] = cur_alg_t_stats
            dataset_alg_t_stats.append(cur_d_time_stats)

        for didx, d in enumerate(datasets):
            with open(os.path.join(datasets_res_dir[d], '{}_meas_stats.json'.format(d)), 'w') as f:
                json.dump(dataset_alg_t_stats[didx], f, indent=2)

    import subprocess as s
    s.call(['notify-send', 'rpg_trajectory_evaluation finished',
            'results in: {0}'.format(os.path.abspath(output_dir))])
    print("#####################################")
    print("<<< Finished.")
    print("#####################################")
