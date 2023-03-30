#!/usr/bin/env python2
"""
@author: Christian Forster
"""

import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import rc
rc('font', **{'family': 'serif', 'serif': ['Cardo']})
rc('text', usetex=True)

from scipy.spatial import KDTree
from webcolors import (
    CSS3_HEX_TO_NAMES,
    hex_to_rgb,
)

FORMAT = '.pdf'


def color_box(bp, color):
    elements = ['medians', 'boxes', 'caps', 'whiskers']
    # Iterate over each of the elements changing the color
    for elem in elements:
        [plt.setp(bp[elem][idx], color=color, linestyle='-', lw=1.0)
         for idx in range(len(bp[elem]))]
    return

def boxplot_compare_abs(ax, xlabels,
                    data, data_labels, data_colors,
                    legend=True):
    n_data = len(data)
    n_xlabel = len(xlabels)
    leg_handles = []
    leg_labels = []
    idx = 0
    for idx, d in enumerate(data):
        # print("idx and d: {0} and {1}".format(idx, d))
        w = 1 / (1.5 * n_data + 1.5)
        widths = [w for pos in np.arange(n_xlabel)]
        positions = [pos - 0.5 + 1.5 * w + idx * w
                     for pos in np.arange(n_xlabel)]
        # print("Positions: {0}".format(positions))
        d_abs = []
        for idx_,d_ in enumerate(d):
            d_abs.append(d_*xlabels[idx_])
        bp = ax.boxplot(d_abs, 0, '', positions=positions, widths=widths)
        color_box(bp, data_colors[idx])
        tmp, = plt.plot([1, 1], c=data_colors[idx], alpha=0)
        leg_handles.append(tmp)
        leg_labels.append(data_labels[idx])
        idx += 1

    ax.set_xticks(np.arange(n_xlabel))
    ax.set_xticklabels(xlabels)
    xlims = ax.get_xlim()
    ax.set_xlim([xlims[0]-0.1, xlims[1]-0.1])
    ax.grid(axis = 'y', linestyle = '--', linewidth = 0.5)
    if legend:
       leg = ax.legend(data_labels)
       for i, j in enumerate(leg.legendHandles):
            j.set_color(data_colors[i])
    map(lambda x: x.set_visible(False), leg_handles)

def boxplot_compare(ax, xlabels,
                    data, data_labels, data_colors,
                    legend=True):
    n_data = len(data)
    n_xlabel = len(xlabels)
    leg_handles = []
    leg_labels = []
    idx = 0
    for idx, d in enumerate(data):
        # print("idx and d: {0} and {1}".format(idx, d))
        w = 1 / (1.5 * n_data + 1.5)
        widths = [w for pos in np.arange(n_xlabel)]
        positions = [pos - 0.5 + 1.5 * w + idx * w
                     for pos in np.arange(n_xlabel)]
        # print("Positions: {0}".format(positions))
        bp = ax.boxplot(d, 0, '', positions=positions, widths=widths)
        color_box(bp, data_colors[idx])
        tmp, = plt.plot([1, 1], c=data_colors[idx], alpha=0)
        leg_handles.append(tmp)
        leg_labels.append(data_labels[idx])
        idx += 1

    ax.set_xticks(np.arange(n_xlabel))
    ax.set_xticklabels(xlabels)
    xlims = ax.get_xlim()
    ax.set_xlim([xlims[0]-0.1, xlims[1]-0.1])
    ax.grid(axis = 'y', linestyle = '--', linewidth = 0.5)
    if legend:
       leg = ax.legend(data_labels)
       for i, j in enumerate(leg.legendHandles):
            j.set_color(data_colors[i])
    map(lambda x: x.set_visible(False), leg_handles)

def boxplot_compare_cpu(ax, xlabels,
                    data, data_labels, data_colors,
                    legend=True):
    n_data = len(data)
    n_xlabel = len(xlabels)
    leg_handles = []
    leg_labels = []
    idx = 0
    for idx, d in enumerate(data):
        # print("idx and d: {0} and {1}".format(idx, d))
        w = 1 / (1.5 * n_data + 1.5)
        widths = [w for pos in np.arange(n_xlabel)]
        positions = [pos - 0.5 + 1.5 * w + idx * w
                     for pos in np.arange(n_xlabel)]
        # print("Positions: {0}".format(positions))
        bp = ax.boxplot(d, 0, '', positions=positions, widths=widths)
        color_box(bp, data_colors[idx])
        tmp, = plt.plot([1, 1], c=data_colors[idx], alpha=0)
        leg_handles.append(tmp)
        leg_labels.append(data_labels[idx])
        idx += 1

    ax.set_xticks(np.arange(n_xlabel))
    ax.set_xticklabels(xlabels)
    ax.grid(axis = 'y', linestyle = '--', linewidth = 0.5)
    if legend:
       leg = ax.legend(data_labels)
       for i, j in enumerate(leg.legendHandles):
            j.set_color(data_colors[i])
    map(lambda x: x.set_visible(False), leg_handles)

def plot_mem_over_time_all(fig, mem_usages, proc_names, data_colors, data_labels, alpha=1.0):
    proc_idx = 0
    for proc_idx in range(len(proc_names[0])-1):
        # create a new subplot
        idx = 0
        if(proc_idx == 0):
            ax = fig.add_subplot(3,3,proc_idx+1, xlabel="Time [s]", ylabel="MEM Usage [\%]")
        else:
            ax = fig.add_subplot(3,3,proc_idx+1, xlabel="Time [s]")

        ax.title.set_text(proc_names[0][proc_idx + 1])
        for idx in range(len(mem_usages)):
            plot_mem_over_time(ax, mem_usages[idx][:,0], mem_usages[idx][:,proc_idx+1], data_colors[idx], data_labels[idx])
            idx += 1
        proc_idx += 1


def plot_mem_over_time(ax, timestamps, mem_usage, color, name, alpha=1.0):
    # substract first timestamp from all times to get relative time
    time_start = timestamps[0]
    timestamps = [(time - time_start)/pow(10,9) for time in timestamps]
    ax.plot(timestamps, mem_usage, color=color, linestyle='-', alpha=alpha, label=name)

def plot_trajectory_top(ax, pos, color, name, alpha=1.0):
    ax.grid(ls='--', color='0.7')
    # pos_0 = pos - pos[0, :]
    ax.plot(pos[:, 0], pos[:, 1], color, linestyle='-', alpha=alpha, label=name)


def plot_trajectory_side(ax, pos, color, name, alpha=1.0):
    ax.grid(ls='--', color='0.7')
    # pos_0 = pos - pos[0, :]
    ax.plot(pos[:, 0], pos[:, 2], color, linestyle='-', alpha=alpha, label=name)


def plot_aligned_top(ax, p_gt, p_es, n_align_frames):
    if n_align_frames <= 0:
        n_align_frames = p_es.shape[0]
    # p_es_0 = p_es - p_gt[0, :]
    # p_gt_0 = p_gt - p_gt[0, :]
    # ax.plot(p_es[0:n_align_frames, 0], p_es[0:n_align_frames, 1],
        # 'g-', linewidth=2, label='aligned')
    for (x1, y1, z1), (x2, y2, z2) in zip(
            p_es[:n_align_frames:10, :], p_gt[:n_align_frames:10, :]):
        ax.plot([x1, x2], [y1, y2], '-', color="gray")


def plot_error_n_dim(ax, distances, errors, results_dir,
                     colors=['r', 'g', 'b'],
                     labels=['x', 'y', 'z']):
    assert len(colors) == len(labels)
    assert len(colors) == errors.shape[1]
    for i in range(len(colors)):
        ax.plot(distances, errors[:, i],
                colors[i]+'-', label=labels[i])

def convert_rgb_to_names(rgb_tuple):

    # a dictionary of all the hex and their respective names in css3
    css3_db = CSS3_HEX_TO_NAMES
    names = []
    rgb_values = []
    for color_hex, color_name in css3_db.items():
        names.append(color_name)
        rgb_values.append(hex_to_rgb(color_hex))

    rgb_tuple = (rgb_tuple[0]*255, rgb_tuple[1]*255, rgb_tuple[2]*255)
    kdt_db = KDTree(rgb_values)
    distance, index = kdt_db.query(rgb_tuple)
    return f'{names[index]}'

def boxplot_compare_freq(ax,
                    data, data_labels, data_colors,
                    legend=True):
    n_data = len(data)
    leg_handles = []
    leg_labels = []
    idx = 0

    for idx, d in enumerate(data):
        w = 1 / (1.5 * n_data + 1.5)
        widths = [w]
        positions = [1.5 * w + idx * w]
        # print("Positions: {0}".format(positions))
        bp = ax.boxplot(d, 0, '', positions=positions, widths=widths)
        color_box(bp, data_colors[idx])
        tmp, = plt.plot([1, 1], c=data_colors[idx], alpha=0)
        leg_handles.append(tmp)
        leg_labels.append(data_labels[idx])
        idx += 1

    ax.tick_params(
        axis='x',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom=False,      # ticks along the bottom edge are off
        top=False,         # ticks along the top edge are off
        labelbottom=False) # labels along the bottom edge are off

    ax.grid(axis = 'y', linestyle = '--', linewidth = 0.5)

    if legend:
       leg = ax.legend(data_labels)
       for i, j in enumerate(leg.legendHandles):
            j.set_color(data_colors[i])
    map(lambda x: x.set_visible(False), leg_handles)
