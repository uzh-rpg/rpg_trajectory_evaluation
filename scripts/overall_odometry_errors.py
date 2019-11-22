#!/usr/bin/env python2

import os
import numpy as np
import matplotlib.pyplot as plt

import add_path
import results_writer as res_writer
import plot_utils as pu

from analyze_trajectories import FORMAT, PLOT_LABELS, COLORS

# boxplot distances that will be used for all datasets for overall errors
OVERALL_BOXPLOT_DISTANCES = [7.0, 14.0, 21.0, 28.0, 35.0]

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
