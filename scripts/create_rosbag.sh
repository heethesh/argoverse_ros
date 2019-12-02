#!/bin/bash

DATASET_DIR="${1}"
LOG_ID="${2}"
OUTPUT_DIR="${3}"
PKG_PATH="${4}"
CAMERAS="${5}"

# Exit on any error
set -e

cd "${PKG_PATH}/scripts"

# This script adds all topics EXCEPT images
python3.6 create_rosbag.py \
    --dataset_dir "${DATASET_DIR}" \
    --log_id "${LOG_ID}" \
    --output_dir "${OUTPUT_DIR}" \
    --cameras "${CAMERAS}"

# This scripts copies all old topics AND appends images to the final bag
python2.7 create_rosbag_images.py \
    --log_id "${LOG_ID}" \
    --output_dir "${OUTPUT_DIR}"

# Clean up intermediate files
cd "${OUTPUT_DIR}"
rm "${LOG_ID}_no_images.bag"
