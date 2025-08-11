#!/bin/bash

set -e

error_exit()
{
    echo "There was an error running python"
    exit 1
}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Setup ROS2 environment
source /opt/ros/humble/setup.bash

# Source the workspace
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/install/setup.bash"
else
    echo "Warning: install/setup.bash not found. Make sure to build the workspace first."
fi

# Use system python by default, but allow overriding
python_exe=${PYTHONEXE:-"python3"}

# Check if we are running in a conda environment
if ! [[ -z "${CONDA_PREFIX}" ]]; then
  echo "Warning: running in conda env, please deactivate before executing this script"
  echo "If conda is desired please source setup_conda_env.sh in your python 3.10 conda env and run python normally"
fi

# Show icon if not running headless
export RESOURCE_NAME="FirefighterSim"

# Execute python with all arguments
$python_exe "$@" || error_exit 