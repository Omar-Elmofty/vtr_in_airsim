
# VT&R in AirSim

This repository is home for the required packages and instructions to run VT&R in AirSim

**Top-level Contents**



## Reference

Check out this guide created by Jacopo  for [link](https://github.com/JacopoPan/a-minimalist-guide/blob/master/Part3-Using-AirSim.md)


## Installing Unreal Engine

Check out the documentation Available Here [link](https://microsoft.github.io/AirSim/build_linux/)

Here is a summary of the instructions for setting up 

Make sure you are registered with Epic Games. This is required to get source code access for Unreal Engine.

Clone Unreal in your favorite folder and build it (this may take a while!). Note: We only support Unreal >= 4.22 at present. We recommend using 4.24.

`git clone -b 4.24 https://github.com/EpicGames/UnrealEngine.git
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make`

Note if you are using Ubuntu 16.04, you might need to install vulkan drivers

`sudo apt install -y libvulkan1 vulkan-utils`


## Installing AirSim

Install AirSim from this Forked Repository

`git clone https://github.com/Omar-Elmofty/AirSim.git
cd AirSim
./setup.sh
./build.sh`

This will build a plugins folder that can be appended to an unreal enviroment

## Install AirSim ROS Wrapper

Instructions are available here [link](https://microsoft.github.io/AirSim/airsim_ros_pkgs/)

You'll need gcc >= 8.0.0, install it if you don't have it
`sudo apt-get install gcc-8 g++-8`

If you encounter issues during install you could run
`sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-8 g++-8`


You'll also need C-make >= 3.10.0, which is not installed by default with VT&R

To install the latest C-make version follow the below instructions

Visit https://cmake.org/download/ and download the latest binaries
	I used `cmake-3.17.3-Linux-x86_64.sh`, copy the binary to /opt/
`chmod +x /opt/cmake-3.17.3-Linux-x86_64.sh` 
`sudo bash /opt/cmake-3.17.3-Linux-x86_64.sh*` (you'll need to press y twice)

The script installs to `/opt/cmake-3.17.3-Linux-x86_64*` so in order to get the cmake command, make a symbolic link:

`sudo ln -s /opt/cmake-3.17.3-Linux-x86_64/bin/* /usr/local/bin`

Test your results with `cmake --version`

Install tf2 sensor and mavros packages: 

`sudo apt-get install ros-kinetic-tf2-sensor-msgs ros-kinetic-mavros*`


To build Ros Wrapper

Open a new terminal

Export C-make paths
`export PATH=/opt/cmake-3.17.2-Linux-x86_64/bin:$PATH`
`export CMAKE_PREFIX_PATH=/opt/cmake-3.17.2-Linux-x86_64:$CMAKE_PREFIX_PATH`

`cd ~AirSim/ros
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8`


## Setting Up Enviroment in Unreal

You can setup an enviroment by downloading it from Epic Launcher and following the instructions in this video [link](https://www.youtube.com/watch?v=1oY8Qu5maQQ&t=305s). You will need Unreal Engine to be installed on a windows machine though, and the files will have to be transfered

Or Pre-compiled Enviroments can be downloaded from here

`$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Africa.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Blocks.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Building_99.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/LandscapeMountains.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Neighborhood.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/SoccerField.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/TrapCam.zip.001
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/TrapCam.zip.002
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Zhangjiajie.zip`


## Setup Python Client

`cd ~/AirSim/PythonClient
python setup.py install`


## Running VT&R in AirSim


Copy `stereo.yaml` in this repo and paste in:

`~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/src/babelfish_robochunk_translator/param/ros_to_rig_images`


Open `m600_backyard.yaml` located in the below directory

`~/charlottetown/utiasASRL/vtr2/src/asrl__navigation/param/scenarios`

Ensure you are using gray-scale images, as follows:

`converter/extraction/conversions: ["RGB_TO_GRAYSCALE"]
converter/extraction/extractor/channels: ["grayscale"]`


