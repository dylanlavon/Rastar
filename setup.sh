#!/bin/bash

# Define the venv directory name
VENV_DIR="rastar-venv"

echo "=========================================="
echo "Initializing Rover A* Simulation Setup"
echo "=========================================="

# 1. Check if the venv already exists
if [ -d "$VENV_DIR" ]; then
    echo "Found existing virtual environment '$VENV_DIR'."
else
    echo "Creating new virtual environment: $VENV_DIR..."
    python3 -m venv $VENV_DIR
fi

# 2. Activate the environment
echo "Activating $VENV_DIR..."
source $VENV_DIR/bin/activate

# 3. Upgrade pip to avoid annoying warning messages
echo "Upgrading pip..."
pip install --upgrade pip

# 4. Install the required packages
if [ -f "requirements.txt" ]; then
    echo "Installing dependencies from requirements.txt..."
    pip install -r requirements.txt
else
    echo "ERR: requirements.txt not found. Skipping dependency installation."
fi

echo "=========================================="
echo "Setup Complete!"
echo "To activate your environment, run:"
echo "source $VENV_DIR/bin/activate"
echo "=========================================="