#!/bin/bash

# Get the directory of the script
WORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd | tr -d '[:space:]')"

# Define the required paths
export SCENARIO_RUNNER_ROOT="${WORK_DIR}/src/external/scenario_runner"
export AGENTS_ROOT="${WORK_DIR}/src/external/scenario_runner/srunner/tests"
export LEADERBOARD_ROOT="${WORK_DIR}/src/external/leaderboard"
export PROJECT_ROOT="${WORK_DIR}/src/tum_agents"
export AW_AGENT_ROOT="${WORK_DIR}/src/tum_agents/autoware_agent"
export CONFIG_ROOT="${WORK_DIR}/config"
export ORIGINAL_PYTHON_PATH="/usr/local/lib/python$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')/site-packages"

# Update PYTHONPATH
export PYTHONPATH="${ORIGINAL_PYTHON_PATH}:${PYTHONPATH}:${SCENARIO_RUNNER_ROOT}:${AGENTS_ROOT}:${LEADERBOARD_ROOT}:${PROJECT_ROOT}:${CONFIG_ROOT}:${AW_AGENT_ROOT}"

# Print the updated PYTHONPATH for verification
echo "PYTHONPATH updated"
