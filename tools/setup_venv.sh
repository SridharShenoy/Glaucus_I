#!/usr/bin/env bash
# Create .venv with everything needed to run the repo without ROS.
#   ./tools/setup_venv.sh && source .venv/bin/activate
set -euo pipefail
cd "$(dirname "$0")/.."
PY="${PYTHON:-python3}"
"$PY" -m venv .venv
.venv/bin/python -m pip install --quiet --upgrade pip
.venv/bin/python -m pip install -r requirements-dev.txt
echo "done. activate with:  source .venv/bin/activate"
