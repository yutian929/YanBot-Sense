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

cd grounded_sam2/

make build-image
check_success

cd ..
cd ../../
check_success

# for file pipe
mkdir -p /tmp/file_pipe/
sudo chmod 777 /tmp/file_pipe/
check_success
echo "read" > /tmp/file_pipe/image.flag
check_success


echo "Next, you need to run docker-grounded_sam2 by yourself."
echo "Tape \`make run\` at src/perception/grounded_sam2/"
echo "Then \`cd /home/appuser/Grounded-SAM-2\`"
echo "Run \`bash scripts/install_deps_xxx.sh\`"
echo "More Instructions can be found in src/perception/grounded_sam2/REAMDE.md ### Installation with docker"
