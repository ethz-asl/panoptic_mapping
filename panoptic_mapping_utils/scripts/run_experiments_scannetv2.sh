#!/bin/bash

MAPPER_CONFIGS=
SCANS_DIR=
SCANS=
OUT_DIR=

HELP_MSG="Usage: run_experiments_scannetv2.sh -c <PATH_TO_MAPPER_CONFIGS_FILE> -d <PATH_TO_SCANS_DIR> -o <OUT_DIR> -s <PATH_TO_SCANS_FILE>"

# Parse CLI options
while getopts ":hc:d:o:s:" opt
do
	case ${opt} in
		h ) 
			echo $HELP_MSG
			exit 0
			;;
		d )
			if [ ! -d $OPTARG ]
			then
				echo "${OPTARG} is not a valid dir path!"
				exit 1
			fi
			SCANS_DIR=$OPTARG
			;;
		c ) 
            if [ ! -f $OPTARG ]
            then
                echo "${OPTARG} is not a valid file path!"
                exit 1
            fi
            MAPPER_CONFIGS=`cat $OPTARG`
			;;
		o )
			OUT_DIR=$OPTARG
			;;
        s )
            if [ ! -f $OPTARG ]
            then
                echo "${OPTARG} is not a valid file path!"
                exit 1
            fi
			SCANS=`cat $OPTARG`
			;;
		\? )
			echo "Invalid Option: -$OPTARG" 1>&2
			exit 1
			;;
	esac
done
shift $((OPTIND -1))

# Validate arguments
: ${SCANS_DIR:? Path to scans dir must be provided!}
: ${MAPPER_CONFIGS:?Mapper configs must be provided!}
: ${SCANS:?Scans must be provided!}
: ${OUT_DIR:?Out dir must be provided!}

# Move to workspace root and source setup.bash
cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null
cd "$( catkin locate )"
. ./devel/setup.bash

for SCAN in $SCANS
do 
    SCAN_DIR=${SCANS_DIR}/${SCAN}
    echo "Running experiments for scan $SCAN"
    for CONFIG in $MAPPER_CONFIGS
    do  
        RUN_OUT_DIR=${OUT_DIR}/${SCAN}/${CONFIG}
        mkdir -p $RUN_OUT_DIR
        CONFIG_YAML_FILE=`find . -name ${CONFIG}.yaml`

        run_dir=$RUN_OUT_DIR yq -i ".data_writer.log_data_writer.output_directory = env(run_dir)" $CONFIG_YAML_FILE

        if [[ $CONFIG == *"groundtruth"* ]]
        then
            LABELS_CSV_FILES=${SCAN_DIR}/${SCAN}_labels.csv
            labels=$LABELS_CSV_FILES yq -i ".labels.file_name = env(labels)" $CONFIG_YAML_FILE
        fi

        # Run panoptic mapper
        roslaunch panoptic_mapping_ros run.launch dataset:=scannetv2 config:=$CONFIG scan_id:=$SCAN

        # Export labeled pointclouds
        roslaunch panoptic_mapping_utils evaluate_series.launch map_file:=$RUN_OUT_DIR ground_truth_pointcloud_file:=${SCANS_DIR}/${SCAN}/${SCAN}.pointcloud.ply output_suffix:=$CONFIG
    done
done
