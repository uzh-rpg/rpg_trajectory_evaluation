from setuptools import find_packages, setup

setup(
    name='rpg_trajectory_evaluation',
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    scripts=[
        'scripts/add_path.py',
        'scripts/analyze_trajectories.py',
        'scripts/analyze_trajectory_single.py',
        'scripts/change_eval_cfg_recursive.py',
        'scripts/fn_constants.py',
        'scripts/overall_odometry_errors.py',
        'scripts/recursive_clean_results_dir.py',
        'scripts/dataset_tools/asl_groundtruth_to_pose.py',
        'scripts/dataset_tools/bag_to_pose.py',
        'scripts/dataset_tools/stamp_state_est.py',
        'scripts/dataset_tools/stamp_state_est_using_matches.py',
        'scripts/dataset_tools/strip_gt_id.py',
        'scripts/dataset_tools/transform_trajectory.py',
    ],
    install_requires=[
        'matplotlib',
        'numpy',
        'scipy',
        'PyYAML',
        'rosbags'
    ],
    python_requires='>=3.6',
    include_package_data=True,
    zip_safe=False,
)
