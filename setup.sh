#!/bin/bash

# Define the venv directory and grab the absolute path of the repository
VENV_DIR="rastar-venv"
REPO_DIR=$(pwd)

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

# 5. Inject the global shortcut into the user's bashrc
echo "Setting up venv alias..."
ALIAS_CMD="alias rastar='source $REPO_DIR/$VENV_DIR/bin/activate'"

if grep -q "alias rastar=" ~/.bashrc; then
    echo "Shortcut 'rastar' already exists in ~/.bashrc. Skipping."
else
    echo "" >> ~/.bashrc
    echo "# Rastar A* Simulation Venv Shortcut" >> ~/.bashrc
    echo "$ALIAS_CMD" >> ~/.bashrc
    echo "Success! Shortcut 'rastar' added."
fi

echo "=========================================="
echo "Setup Complete!"
echo "To use your new shortcut right now, refresh your terminal by running:"
echo "source ~/.bashrc"
echo ""
echo "Then, type 'rastar' from anywhere to activate the environment."
echo "=========================================="