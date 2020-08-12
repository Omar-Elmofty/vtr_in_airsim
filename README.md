
# VT&R in AirSim

This repository is home for the required packages and instructions to run VT&R in AirSim

**Top-level Contents**
* [Install Unreal Engine](#UnrealEngine)
* [Install AirSim](#InstallAirSim)
* [Install AirSim ROS Wrapper](#InstallRos)
* [Setup Python Client](#SettingPython)
* [Setup Unreal Enviroment](#SettingEnv)
* [Prepare VT&R for AirSim](#SettingVTR)
* [Run VT&R in AirSim](#RunningVTR)
* [Reference Material](#Reference)

## Install Unreal Engine <a name="UnrealEngine"></a>

To install Unreal Engine, clone the repo from Epic games and build it. Make sure you create an EpicGames Account First to be able to access the repo.

```
cd ~
git clone -b 4.24 https://github.com/EpicGames/UnrealEngine.git
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```

To launch Unreal Editor to test installation:
```
cd ~/UnrealEngine/Engine/Binaries/Linux/
./UE4Editor
```
The new project Window should open, you can just create an empty project to test everything works fine. 

**Important Note**, in order to avoid Unreal Engine from dropping in performance while running VT&R, ensure it can run in the background.

Go to "Edit -> Editor Preferences"; in the "Search" box, type "CPU" and ensure that the "Use Less CPU when in Background" is unchecked

Note that Unreal Engine uses Vulkan Drivers, if you are using Ubuntu 16.04 they might not be installed by default. If you receive a warning message "opengl is deprecated please use vulkan" upon launching Unreal Engine, then you need to install vulkan using the below command.

`sudo apt install -y libvulkan1 vulkan-utils`


## Install AirSim <a name="InstallAirSim"></a>

Install the AirSim for the forked repo, the forked repo has changes are not available on AirSim's master branch. All the changes to the airsim master branch are presented in [AirSim_Changes.md](https://github.com/Omar-Elmofty/VT-R_in_AirSim/blob/master/README.md)
```
git clone https://github.com/Omar-Elmofty/AirSim.git
cd AirSim
./setup.sh
./build.sh
```


This will build a plugins folder that can be appended to an unreal enviroment

## Install AirSim ROS Wrapper <a name="InstallRos"></a>

Instructions are available here [link](https://microsoft.github.io/AirSim/airsim_ros_pkgs/)

You'll need gcc >= 8.0.0, install it if you don't have it
`sudo apt-get install gcc-8 g++-8`

If you encounter issues during install you could run
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-8 g++-8
```


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

```
Export C-make paths
export PATH=/opt/cmake-3.17.2-Linux-x86_64/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/cmake-3.17.2-Linux-x86_64:$CMAKE_PREFIX_PATH
```

Build AirSim
```
cd ~AirSim/ros
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```


## Setup Python Client <a name="SettingPython"></a>
```
cd ~/AirSim/PythonClient
python setup.py install
```


## Setup Unreal Enviroment <a name="SettingEnv"></a>

You can setup an enviroment by downloading it from Epic Launcher and following the instructions in this video [link](https://www.youtube.com/watch?v=1oY8Qu5maQQ&t=305s). You will need Unreal Engine to be installed on a windows machine though, and the files will have to be transfered

Or Pre-compiled Enviroments can be downloaded from here
```
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Africa.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Blocks.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Building_99.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/LandscapeMountains.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Neighborhood.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/SoccerField.zip
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/TrapCam.zip.001
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/TrapCam.zip.002
$ wget https://github.com/microsoft/AirSim/releases/download/v1.3.1-linux/Zhangjiajie.zip
```



## Prepare VT&R for AirSim <a name="SettingVTR"></a>


Copy `stereo.yaml` in this repo and paste in:

```
~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/src/babelfish_robochunk_translator/param/ros_to_rig_images
```


Open `m600_backyard.yaml` located in the below directory

`~/charlottetown/utiasASRL/vtr2/src/asrl__navigation/param/scenarios`

Ensure you are using gray-scale images, as follows:

```
converter/extraction/conversions: ["RGB_TO_GRAYSCALE"]
converter/extraction/extractor/channels: ["grayscale"]
```


## Run VT&R in AirSim <a name="RunningVTR"></a>



## Reference Material <a name="Reference"></a>

[Official AirSim Documentation](https://microsoft.github.io/AirSim)
[AirSim Installation Instructions for Linux](https://microsoft.github.io/AirSim/build_linux/)
[AirSim ROS Wrapper Documentation](https://microsoft.github.io/AirSim/airsim_ros_pkgs/)
[AirSim Settings Documentation](https://microsoft.github.io/AirSim/settings/)
[Great AirSim Guide Created by Jacopo Panerati Part1](https://github.com/JacopoPan/a-minimalist-guide/blob/master/Part3-Using-AirSim.md)
[Great AirSim Guide Created by Jacopo Panerati Part2](https://github.com/JacopoPan/a-minimalist-guide/blob/master/Part4-Modifying-AirSim.md)
