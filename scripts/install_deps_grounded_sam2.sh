#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# Clone grounded_sam2
mkdir -p src/perception/
cd src/perception/

if [ -d "grounded_sam2" ]; then
    echo "grounded_sam2 directory already exists. Skipping clone."
else
    echo "Cloning grounded_sam2..."
    git clone https://github.com/yutian929/YanBot-Sense_Grounded_SAM_2.git grounded_sam2
    check_success
fi

echo "Next, you need to build docker-grounded_sam2 by yourself."
echo "Instructions can be found in src/perception/grounded_sam2/REAMDE.md ### Installation with docker"


cd ../../
