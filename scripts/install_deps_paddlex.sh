#!/bin/bash

# Function to check command success
check_success() {
    if [ $? -ne 0 ]; then
        echo "Error occurred in the previous command. Exiting."
        exit 1
    fi
}

# Clone PaddleX
mkdir -p src/perception/
cd src/perception/

if [ -d "paddlex" ]; then
    echo "paddlex directory already exists. Skipping clone."
else
    echo "Cloning paddlex..."
    git clone https://github.com/PaddlePaddle/PaddleX.git paddlex
    check_success
fi

echo "Next, you need to build docker-paddlex by yourself."
echo "Instructions can be found in src/perception/paddlex/REAMDE.md"


cd ../../