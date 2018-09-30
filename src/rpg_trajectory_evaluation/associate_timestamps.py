#!/usr/bin/env python2


def read_timestamps(filename):
    """
    assume the first column of the file contains timestamp in second
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    data_list = [[v.strip() for v in line.split(" ") if v.strip() != ""]
                 for line in lines if len(line) > 0 and line[0] != "#"]
    stamps = [float(v[0]) for v in data_list]
    return stamps


def associate(first_stamps, second_stamps, offset, max_difference):
    """
    associate timestamps

    first_stamps, second_stamps: list of timestamps to associate

    Output:
    sorted list of matches (match_first_idx, match_second_idx)
    """
    potential_matches = [(abs(a - (b + offset)), idx_a, idx_b)
                         for idx_a, a in enumerate(first_stamps)
                         for idx_b, b in enumerate(second_stamps)
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()  # prefer the closest
    matches = []
    first_idxes = range(len(first_stamps))
    second_idxes = range(len(second_stamps))
    for diff, idx_a, idx_b in potential_matches:
        if idx_a in first_idxes and idx_b in second_idxes:
            first_idxes.remove(idx_a)
            second_idxes.remove(idx_b)
            matches.append((int(idx_a), int(idx_b)))

    matches.sort()
    return matches


def read_files_and_associate(first_file, second_file, offset, max_diff):
    first_stamps = read_timestamps(first_file)
    second_stamps = read_timestamps(second_file)

    return associate(first_stamps, second_stamps, offset, max_diff)

if __name__ == '__main__':
    pass
