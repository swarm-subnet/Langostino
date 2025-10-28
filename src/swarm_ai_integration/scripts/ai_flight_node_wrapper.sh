#!/bin/bash
# Wrapper script to run ai_flight_node.py with Python virtual environment

# Path to your virtual environment (adjust this to your venv location)
VENV_PATH="${HOME}/ai_flight_node_env"  # Change this to your actual venv path

# Activate the virtual environment
source "${VENV_PATH}/bin/activate"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Run the ai_flight_node with the venv's Python interpreter
exec "${VENV_PATH}/bin/python3" "${SCRIPT_DIR}/ai_flight_node.py" "$@"

# Note: exec replaces this shell process, so deactivate is automatic on exit
