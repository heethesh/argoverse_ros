#!/bin/bash

DATASET_DIR="/home/heethesh/Datasets/Argoverse/argoverse-tracking/sample"
LOG_ID="c6911883-1843-3727-8eaa-41dc8cda8993"

# This script adds all topics EXCEPT images
python3.6 create_rosbag.py \
    --dataset_dir "${DATASET_DIR}" \
    --log_id "${LOG_ID}" \
    --output_dir ./ \
    --cameras ring_front_center ring_front_left

# This scripts copies all old topics AND appends images to the final bag
python2.7 create_rosbag_images.py \
    --log_id "${LOG_ID}" \
    --output_dir ./

# Clean up intermediate files
if [ $? -eq 0 ]; then
    rm "${LOG_ID}_no_images.bag"
fi
