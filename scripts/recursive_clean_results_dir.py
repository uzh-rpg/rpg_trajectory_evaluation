#!/usr/bin/env python

import os
import argparse
import shutil
from colorama import init, Fore

init(autoreset=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root_dir', type=str, help='root folder to iterate')
    parser.add_argument('--res_dir', type=str, default='saved_results')
    args = parser.parse_args()

    print(Fore.YELLOW + "Going to remove all the results dir {} in {}".format(
        args.res_dir, args.root_dir))

    for dirpath, dirnames, filenames in os.walk(args.root_dir):
        if args.res_dir in dirnames:
            dir_fn = os.path.join(dirpath, args.res_dir)
            print(Fore.RED + "Remove {}".format(dir_fn))
            shutil.rmtree(dir_fn)
