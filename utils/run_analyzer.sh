#!/bin/sh
set -e
function main
{
	# Get command line options
    local CONF_YAML=~/git/eth/paul-joseph/Data/Testing_Configs/euroc_test.yaml
    local OUT_DIR="./results/euroc_ofvins_mono_Pi"
    local RESULTS_DIR="/home/paul/git/eth/paul-joseph/Data/Testing/EuRoC/"
    local GT_DIR="/home/paul/git/eth/paul-joseph/Data/GroundTruth/EuRoC/"

	while getopts ":ho:r:g:c:" opt; do
	    case $opt in
	        h)
	            print_help
	            exit 0
	            ;;
	        o)
	        	OUT_DIR=$OPTARG
	            ;;
		    r)
		    	RESULTS_DIR=$OPTARG
		    	;;
		    g)
		    	GT_DIR=$OPTARG
		    	;;
		    c)
		    	CONF_YAML=$OPTARG
		    	;;
	        *)
	         	exit_fail "Illegal parameter!"
	         	;;
	    esac
	done

	echo -e 'Using Config     	: ' $CONF_YAML
	echo -e 'Ground Truth Path  : ' $GT_DIR
	echo -e 'Results Path       : ' $RESULTS_DIR
	echo -e 'Output Path        : ' $OUT_DIR

    run_python_script

	exit 0

}

function run_python_script
{
    python scripts/analyze_trajectories.py $CONF_YAML --output_dir=$OUT_DIR --results_dir=$RESULTS_DIR --gt_dir=$GT_DIR --odometry_error_per_dataset --overall_odometry_error --plot_trajectories --rmse_table --no_plot_aligned --no_plot_traj_per_alg --plot_system_logs
}

function print_help
{
	echo
	echo "This script uses ffmpeg to record the main screen"
	echo
	echo "Usage:"
	echo
	echo "	$(basename "$0") [-h]  |  -o <file_name>"
	echo
	echo "Where:"
	echo
	echo "	-h	Display help and exit"
	echo "	-o	name of output dir"
	echo "	-r	name of results dir"
    echo "	-g	name of ground truth dir"
	echo "	-c	name of the config file"
	echo
}

# Run script
main "$@"

# We should not end up here
exit 42
