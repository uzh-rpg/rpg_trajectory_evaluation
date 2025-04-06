#!/usr/bin/env python3
import os

import numpy as np
import yaml


def compute_statistics(data_vec):
    stats = dict()
    if len(data_vec) > 0:
        stats['rmse'] = float(
            np.sqrt(np.dot(data_vec, data_vec) / len(data_vec)))
        stats['mean'] = float(np.mean(data_vec))
        stats['median'] = float(np.median(data_vec))
        stats['std'] = float(np.std(data_vec))
        stats['min'] = float(np.min(data_vec))
        stats['max'] = float(np.max(data_vec))
        stats['num_samples'] = int(len(data_vec))
    else:
        stats['rmse'] = 0
        stats['mean'] = 0
        stats['median'] = 0
        stats['std'] = 0
        stats['min'] = 0
        stats['max'] = 0
        stats['num_samples'] = 0

    return stats


def update_and_save_stats(new_stats, label, yaml_filename):
    stats = dict()
    if os.path.exists(yaml_filename):
        stats = yaml.load(open(yaml_filename, 'r'), Loader=yaml.FullLoader)
    stats[label] = new_stats

    with open(yaml_filename, 'w') as outfile:
        outfile.write(yaml.dump(stats, default_flow_style=False))

    return


def compute_and_save_statistics(data_vec, label, yaml_filename):
    new_stats = compute_statistics(data_vec)
    update_and_save_stats(new_stats, label, yaml_filename)

    return new_stats


def write_tex_table(list_values, rows, cols, outfn):
    '''
    write list_values[row_idx][col_idx] to a table that is ready to be pasted
    into latex source

    list_values is a list of row values

    The value should be string of desired format
    '''

    assert len(rows) >= 1
    assert len(cols) >= 1

    with open(outfn, 'w') as f:
        # write header
        f.write('      &      ')
        for col_i in cols[:-1]:
            f.write(col_i + ' & ')
        f.write(' ' + cols[-1]+'\n')

        # write each row
        for row_idx, row_i in enumerate(list_values):
            f.write(rows[row_idx] + ' &     ')
            row_values = list_values[row_idx]
            for col_idx in range(len(row_values) - 1):
                f.write(row_values[col_idx] + ' & ')
            f.write(' ' + row_values[-1]+' \n')

def save_algorithm_rmse(algorithm_rmse, dataset_names, algorithm_names, n_trials,
                        output_dir):
    n_data = len(dataset_names)
    for v in algorithm_names:
        assert len(algorithm_rmse['trans_err'][v]) == n_data
        assert len(algorithm_rmse['rot_err'][v]) == n_data

    output_names = ["ate_rmse_trans.txt", "ate_rmse_rot.txt"]
    keys = ["trans_err", "rot_err"]
    for index, outputname in enumerate(output_names):
        output_file = os.path.join(output_dir, outputname)
        with open(output_file, 'w') as stream:
            stream.write("#Rows correspond to algorithms")
            for v in algorithm_names:
                stream.write(',{}'.format(v))
            stream.write(', on {} datasets x {} trials. Missing RMSE is filled with inf.\n'.format(
                n_data, n_trials))
            for v in algorithm_names:
                sessiontrials = algorithm_rmse[keys[index]][v]
                for session in sessiontrials:
                    pad = n_trials - len(session)
                    for j in range(pad):
                        stream.write(' inf')
                    for trial in session:
                        stream.write(' {}'.format(trial))
                stream.write('\n')
            stream.write('\n')