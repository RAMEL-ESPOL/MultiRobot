#!/bin/bash

date=$(date +%Y%m%d)
output_dir="results/"

# test all cc files in src
mkdir -p ${output_dir}
result_fname=${output_dir}${date}".txt"
cppcheck \
    aruco_ros2/src \
    --enable=all \
    -I aruco_ros2/include \
    --suppress=missingIncludeSystem \
    --output-file=$result_fname
