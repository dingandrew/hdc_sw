
#!/bin/bash
#/*-----------------------------------------------------------------
#File name     : env_setup.sh
#Developers    : Andrew Ding
#Created       : 02/2024
#Description   : Setup the enviroment
#
#-------------------------------------------------------------------
#Copyright Andrew Ding 2024
#-----------------------------------------------------------------*/


# Project level Enviroment Setup 

export PRJ_HOME=`git rev-parse --show-toplevel`

# Increase Stack Size Limit
ulimit -s 64000

# Ensure gcc version > 9.0
# Below command is specific to my machine
# export PATH=/opt/rh/devtoolset-11/root/bin/:$PATH

# Create the python venv
# Name of the virtual environment directory
VENV_DIR="env"
# Check if the virtual environment directory exists
if [ -d "$PRJ_HOME/$VENV_DIR" ]; then
    # The directory exists, which means the virtual environment is likely set up
    echo "A virtual environment already exists."

    echo "source $PRJ_HOME/$VENV_DIR/bin/activate"
    source $PRJ_HOME/$VENV_DIR/bin/activate

else
    # The directory does not exist, so we need to create a new virtual environment
    echo "Creating a new virtual environment..."
    
    # Check if Python 3 is installed and available
    if command -v python3.9 &>/dev/null; then
        # Create a new virtual environment using Python 3
        python3.9 -m venv "$VENV_DIR"
        source $PRJ_HOME/$VENV_DIR/bin/activate
        
        echo "Virtual environment created."
    else
        echo "Python 3 is not installed. Please install Python 3 to continue."
        exit 1
    fi
    pip3 install --upgrade pip
fi

if [ -f "./requirements.txt" ]; then
    pip3 install -r ./requirements.txt
else
    echo "No requirements.txt found execute from $PRJ_HOME to re-install dependencies"
fi


echo "ENVIROMENT ACTIVATED"