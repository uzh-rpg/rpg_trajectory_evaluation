#!/usr/bin/evn python

import os
import argparse
import yaml
from colorama import init, Fore

init(autoreset=True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root_dir', type=str, help='root folder to iterate')
    parser.add_argument('align_type', type=str, help='alignment type')
    parser.add_argument('align_n', type=int, help='alignment type')
    args = parser.parse_args()
    print("Arguments:\n{}".format(
        '\n'.join(['-{}: {}'.format(k, v) for k, v in args.__dict__.items()])))

    print(Fore.YELLOW + "Going to change all eval_cfg.yaml under {}.".format(args.root_dir))

    for dirpath, dirnames, filenames in os.walk(args.root_dir):
        if 'eval_cfg.yaml' in filenames:
            eval_cfg_fn = os.path.join(dirpath, 'eval_cfg.yaml')
            print('- Process {}...'.format(eval_cfg_fn))
            with open(eval_cfg_fn, 'r') as f:
                eval_cfg = yaml.load(f, Loader=yaml.FullLoader)
                print("  {} -> {}".format(eval_cfg['align_type'], args.align_type))
                print("  {} -> {}".format(eval_cfg['align_num_frames'], args.align_n))
                eval_cfg['align_type'] = args.align_type
                eval_cfg['align_num_frames'] = args.align_n
            with open(eval_cfg_fn, 'w') as f:
                f.write(yaml.dump(eval_cfg, default_flow_style=False))
