#!/usr/bin/env python2

import os
import argparse

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

import add_path
from trajectory import Trajectory
import plot_utils as pu

rc('font', **{'family': 'serif', 'serif': ['Cardo']})
rc('text', usetex=True)

FORMAT = '.pdf'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='''Analyze trajectory estimate in a folder.''')
    parser.add_argument(
        'result_dir', type=str,
        help="Folder containing the groundtruth and the estimate.")
    parser.add_argument(
        '--plots_dir', type=str,
        help="Folder to output plots",
        default='')
    parser.add_argument('--recalculate_errors',
                        help='Deletes cached errors', action='store_true')
    parser.add_argument('--png',
                        help='Save plots as png instead of pdf',
                        action='store_true')
    args = parser.parse_args()

    assert os.path.exists(args.result_dir)

    plots_dir = args.plots_dir
    if not args.plots_dir:
        plots_dir = os.path.join(args.result_dir, 'plots')
    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir)
    if args.recalculate_errors:
        Trajectory.remove_cached_error(args.result_dir)
    if args.png:
        FORMAT = '.png'

    print("Going to analyze the results in {0}.".format(args.result_dir))
    print("The plots will saved in {0}.".format(plots_dir))

    # compute the errors
    print(">>> Calculating errors...")
    traj = Trajectory(args.result_dir)
    traj.compute_absolute_error()
    traj.compute_relative_errors()
    traj.cache_current_error()
    traj.write_errors_to_yaml()

    # do some plotting
    print(">>> Plotting absolute error...")
    fig = plt.figure(figsize=(6, 5.5))
    ax = fig.add_subplot(111, aspect='equal',
                         xlabel='x [m]', ylabel='y [m]')
    pu.plot_trajectory_top(ax, traj.p_es_aligned, 'b', 'Estimate')
    pu.plot_trajectory_top(ax, traj.p_gt, 'm', 'Groundtruth')
    pu.plot_aligned_top(ax, traj.p_es_aligned, traj.p_gt,
                        traj.align_num_frames)
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    fig.tight_layout()
    fig.savefig(plots_dir+'/trajectory_top' + '_' + traj.align_str +
                FORMAT, bbox_inches="tight")

    fig = plt.figure(figsize=(6, 5.5))
    ax = fig.add_subplot(111, aspect='equal',
                         xlabel='x [m]', ylabel='y [m]')
    pu.plot_trajectory_side(ax, traj.p_es_aligned, 'b', 'Estimate')
    pu.plot_trajectory_side(ax, traj.p_gt, 'm', 'Groundtruth')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    fig.tight_layout()
    fig.savefig(plots_dir+'/trajectory_side' + '_' + traj.align_str +
                FORMAT, bbox_inches="tight")

    fig = plt.figure(figsize=(8, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance [m]', ylabel='Position Drift [mm]',
        xlim=[0, traj.accum_distances[-1]])
    pu.plot_error_n_dim(ax, traj.accum_distances,
                        traj.abs_errors['abs_e_trans_vec']*1000, plots_dir)
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots_dir+'/translation_error' + '_' + traj.align_str
                + FORMAT, bbox_inches="tight")

    fig = plt.figure(figsize=(8, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance [m]', ylabel='Orient. err. [deg]',
        xlim=[0, traj.accum_distances[-1]])
    pu.plot_error_n_dim(
        ax, traj.accum_distances,
        traj.abs_errors['abs_e_ypr']*180.0/np.pi, plots_dir,
        labels=['yaw', 'pitch', 'roll'])
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots_dir+'/rotation_error'+'_'+traj.align_str + FORMAT,
                bbox_inches='tight')

    fig = plt.figure(figsize=(8, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance [m]', ylabel='Scale Drift [\%]',
        xlim=[0, traj.accum_distances[-1]])
    pu.plot_error_n_dim(
        ax, traj.accum_distances,
        np.reshape(traj.abs_errors['abs_e_scale_perc'], (-1, 1)),
        plots_dir, colors=['b'], labels=['scale'])
    ax.legend()
    fig.tight_layout()
    fig.savefig(plots_dir+'/scale_error'+'_'+traj.align_str+FORMAT,
                bbox_inches='tight')

    print(">>> Plotting relative (odometry) error...")
    distances = traj.preset_boxplot_distances
    rel_trans_err = [[traj.rel_errors[d]['rel_trans'] for d in distances]]
    rel_trans_err_perc = [[traj.rel_errors[d]['rel_trans_perc']
                           for d in distances]]
    rel_yaw_err = [[traj.rel_errors[d]['rel_yaw'] for d in distances]]
    labels = ['Estimate']
    colors = ['b']

    fig = plt.figure(figsize=(6, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance traveled [m]',
        ylabel='Translation error [m]')
    pu.boxplot_compare(ax, distances, rel_trans_err, labels, colors)
    fig.tight_layout()
    fig.savefig(plots_dir+'/rel_translation_error'+FORMAT, bbox_inches="tight")
    plt.close(fig)

    fig = plt.figure(figsize=(6, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance traveled [m]',
        ylabel='Translation error [\%]')
    pu.boxplot_compare(ax, distances, rel_trans_err_perc, labels, colors)
    fig.tight_layout()
    fig.savefig(plots_dir+'/rel_translation_error_perc'+FORMAT,
                bbox_inches="tight")
    plt.close(fig)

    fig = plt.figure(figsize=(6, 2.5))
    ax = fig.add_subplot(
        111, xlabel='Distance traveled [m]',
        ylabel='Yaw error [deg]')
    pu.boxplot_compare(ax, distances, rel_yaw_err, labels, colors)
    fig.tight_layout()
    fig.savefig(plots_dir+'/rel_yaw_error'+FORMAT, bbox_inches="tight")
    plt.close(fig)
