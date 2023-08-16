#!/bin/bash

# Get the directory of the current script
script_dir="$(dirname "$(readlink -f "$0")")"

# Navigate to the parent directory
cd "$script_dir/../../../.."


# Source the setup script
source install/setup.bash

# Navigate to the directory
cd "$script_dir"

# Run the Python script
python3 GUI.py

