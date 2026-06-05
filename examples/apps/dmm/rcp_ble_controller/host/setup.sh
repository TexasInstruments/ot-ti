#!/usr/bin/env bash
# Creates a Python virtual environment and installs dependencies.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

echo "Creating virtual environment in $VENV_DIR ..."
python3 -m venv "$VENV_DIR"

echo "Activating and installing dependencies ..."
"$VENV_DIR/bin/pip" install --upgrade pip -q
"$VENV_DIR/bin/pip" install -r "$SCRIPT_DIR/requirements.txt"

echo ""
echo "Done. To activate:"
echo "  source $VENV_DIR/bin/activate"
echo ""
echo "Then run:"
echo "  python spinel_bridge/spinel_bridge.py --port /dev/ttyACM0 --baud 921600"
