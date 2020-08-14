
# VT&R in AirSim

This repository is home for the required packages and instructions to run VT&R in AirSim

**Top-level Contents**
* [Repository Contents](#RepoContents)
* [Install Unreal Engine](#UnrealEngine)
* [Install AirSim](#InstallAirSim)
* [Install AirSim ROS Wrapper](#InstallRos)
* [Setup Python Client](#SettingPython)
* [Setup Unreal Environment](#SettingEnv)
* [Install AirSim Interface](#AirSimInterface)
* [Prepare VT&R for AirSim](#SettingVTR)
* [Run VT&R in AirSim](#RunningVTR)
* [Reference Material](#Reference)


## Repository Contents <a name="RepoContents"></a>

This is a high-level overview of all the contents of this repository

* `Plugins`  *a backup version of the Plugins folder with M600 Implementation folder required to run airsim with Unreal editor*
* `airsim_vtr_interface` *a catkin package for interfacing airsim with VT&R*
* `scripts`  *contains some useful bash scripts*
* `tmuxp`  *contains the yaml files required for running VT&R with airsim*
* `Airsim_Changes.md`  *a document containing all the changes made to airsim's master branch to implement the DJI M600 in it*
* `AirSim_Stereo_Gimbal.md` *a document containing all the details regarding the custom stereo gimbal*
* `settings.json` *the required airsim settings file for running VT&R*
* `stereo.yaml`  *the required parameter file for babelfish_robochunk_translator to accept the airsim topics*

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
```
sudo apt install -y libvulkan1 vulkan-utils
```

## Install AirSim <a name="InstallAirSim"></a>


Install the AirSim for the following forked repo, the forked repo has changes are not available on AirSim's master branch.  All the changes in that repo are presented in the doc [AirSim_Changes.md](https://github.com/Omar-Elmofty/VT-R_in_AirSim/blob/master/AirSim_Changes.md). The changes are primarily adapting the simulator to work with the M600. If you would like to make any changes to the M600 dynamics model, please refer to that doc. 
```
git clone https://github.com/Omar-Elmofty/AirSim.git
cd AirSim
./setup.sh
./build.sh
```
Once Airsim is built, a "Plugins" Folder will be created under `~/AirSim/Unreal`, this folder will be later attached to unreal projects in order to run Airsim with Unreal editor. Note that a backup version of the "Plugins" folder is attached to this repo.


## Install AirSim ROS Wrapper <a name="InstallRos"></a>

#### Upgrade gcc
You'll need gcc >= 8.0.0, check the version of gcc using `gcc --version`. Run the following commands to install gcc 8 if you don't have it:
```
sudo apt-get install gcc-8 g++-8
```
If you encounter issues running the above command, you might need to add ppa the ppa repository as shown in the below commands:

```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-8 g++-8
```
#### Upgrade CMake
You'll also need C-make >= 3.10.0, which is not installed by default with VT&R

To install the latest C-make version follow the below instructions

Visit https://cmake.org/download/ and download the latest binaries
I used `cmake-3.17.3-Linux-x86_64.sh`, copy the binary to /opt/

`chmod +x /opt/cmake-3.17.3-Linux-x86_64.sh` 
`sudo bash /opt/cmake-3.17.3-Linux-x86_64.sh*` (you'll need to press y twice)

The script installs to `/opt/cmake-3.17.3-Linux-x86_64*` so in order to get the cmake command, make a symbolic link:

`sudo ln -s /opt/cmake-3.17.3-Linux-x86_64/bin/* /usr/local/bin`

Test your results with `cmake --version`

If the cmake version does not change, then you might need to export paths as shown below

```
export PATH=/opt/cmake-3.17.2-Linux-x86_64/bin:$PATH
export CMAKE_PREFIX_PATH=/opt/cmake-3.17.2-Linux-x86_64:$CMAKE_PREFIX_PATH
```

#### Install tf2 sensor and mavros
Finally, you'll also need tf2 sensor and mavros packages, install them using the below command:

`sudo apt-get install ros-kinetic-tf2-sensor-msgs ros-kinetic-mavros*`

#### Build ROS Wrapper

To build Ros Wrapper
Open a new terminal, export the cmake paths if you need to, then build ros package

```
cd ~AirSim/ros
catkin build -DCMAKE_C_COMPILER=gcc-8 -DCMAKE_CXX_COMPILER=g++-8
```

## Setup Python Client <a name="SettingPython"></a>

To setup the airsim python package run the following
```
cd ~/AirSim/PythonClient
sudo python setup.py install
```
Now `import airsim` should work in a python script

The different Functions available in the python Client for use can be found in `~/AirSim/pythonClient/airsim/client.py`

Different Examples of the python client usage can be found in `~/Airsim_Forked/PythonClient/multirotor`

## Setup Unreal Environment <a name="SettingEnv"></a>

You can setup an environment by downloading it from Epic Launcher and following the instructions in this video [link](https://www.youtube.com/watch?v=1oY8Qu5maQQ&t=305s). You will need Unreal Engine to be installed on a windows machine though, and the files will have to be transfered

## Install AirSim Interface <a name="AirSimInterface"></a>

The AirSim Interface is a custom Package meant to interface AirSim with VT&R. The package is called `airsim_vtr_interface` and is available in this repo. 

Clone this repo
```
cd ~
git clone https://github.com/Omar-Elmofty/vtr_in_airsim.git
```

Create a catkin workspace
```
mkdir -p ~/airsim_interface/src
cd ~/airsim_interface/
catkin_make
```

Copy and paste package to `airsim_vtr_interface` into new workspace
```
cp -R ~/vtr_in_airsim/airsim_vtr_interface ~/airsim_interface/src
```

Build workspace
```
cd ~/airsim_interface/
catkin_make
```

**Important Note**: the `airsim_vtr_interface` package depends on the DJI SDK ros wrapper `dji_osdk_ros`, depending on which version of the sdk you have, the actual ros wrapper name might differ from `dji_osdk_ros`. If the package name is different, you'll have to change it in `airsim_gimbal_controller.py`, `airsim_interface.py` and `package.xml`.

Also, additional documentation on the custom gimbal stereo which is implemented in `airsim_gimbal_controller.py` can be found in the doc [AirSim_Stereo_Gimbal.md](https://github.com/Omar-Elmofty/VT-R_in_AirSim/blob/master/AirSim_Stereo_Gimbal.md).

## Prepare VT&R for AirSim <a name="SettingVTR"></a>


Copy `stereo.yaml` available in this repo and paste in in the `babelfish_robochunk_translator` package. `stereo.yaml` contains all the correct names of all the ros topics published from the airsim interface. 

```
cp ~/vtr_in_airsim/stereo.yaml ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/src/babelfish_robochunk_translator/param/ros_to_rig_images
```


Open `m600_backyard.yaml` 

`gedit ~/charlottetown/utiasASRL/vtr2/src/asrl__navigation/param/scenarios/m600_backyard.yaml`

Ensure you are using gray-scale images, as follows:

```
converter/extraction/conversions: ["RGB_TO_GRAYSCALE"]
converter/extraction/extractor/channels: ["grayscale"]
```

Finally, you'll have to copy the `settings.json` file in this repo to your Documents folder 
```
cp ~/vtr_in_airsim/settings.json ~/Documents/AirSim
```

The `settings.json` contains all the settings required by AirSim to load the drone and its sensors, the file you just copied contains basics needed to run VT&R, however a lot more customization could be applied to the drone. Check [AirSim Settings Documentation](https://microsoft.github.io/AirSim/settings/) for more details.

## Run VT&R in AirSim <a name="RunningVTR"></a>

#### Step 1 - Load Environment:
Load the Unreal environment by either double clicking the `.uproject` file, or by running it from command line as follows:

```
cd ~/UnrealEngine/Engine/Binaries/Linux/
./UE4Editor PATH_TO_PROJECT/PROJECT_NAME.uproject 

```
Once the Unreal Editor opens, click the **Play** button on the top bar of the Editor (should like music play button). Confirm that you are not receiving any errors in the editor, errors will be highlighted in red.

Also, note that the display in the unreal editor is turned off, that is to help decrease the computational load, and hence increase the camera publishing frequency from airsim. The live feed from the camera will be displayed using `rqt_image_view` package later on. To turn on display again, you could change the ViewMode in `settings.json`, check the airsim documentation for more details.

Note: you could edit the there is `launch_environment.sh` script under `~/vtr_in_airsim/scripts/` and paste the above commands.

#### Step 2 - Launch airsim ros wrapper:
Load `airsim.yaml` using tmuxp. This will initiate the ros wrapper, the stereo images publishers, the airsim gimbal controller, and the rqt image viewer.

```
cd ~/vtr_in_airsim/tmuxp
tmuxp load airsim.yaml
```
You should now see a window pop up with the live camera feed from airsim.

#### Step 3 - Launch VT&R:

Launch VT&R by running the following commands

```
cd ~/vtr_in_airsim/tmuxp
tmuxp load vtr2_m600_airsim.yaml
```

The `vtr2_m600_airsim.yaml` is a slightly modified version of the `vtr2_m600_backyard.yaml` file.

After the file is loaded you should see three panes
* The left pane is running VT&R
* The top right pane is controlling the drone in airsim by running `./airsim_interface.sh`
* The bottom right pane is for initiating learn or return by running `./learn.sh` or `./return.sh`


#### Step 4 - Teach:

Start the teach pass, first you'll have to start moving the drone. Navigate to the top right pane as described in step 3, and run the following command:

```
./airsim_interface.sh
```

The drone now should take off vertically first, then start moving in the horizontal plane in an arc

After the drone finishes taking off, you can start the teach phase by navigating to the bottom right pane, and running the following

```
./learn.sh
```

#### Step 5 - Repeat:
 
Once the top right pane prints the message `Initiating Return Phase Control Loop`, you could then start the return phase by running the following in the bottom right pane:

```
./return.sh
```
Once the top right pane prints `Reached End, Hovering` this indicates the end of the repeat run.


## Reference Material <a name="Reference"></a>

[Official AirSim Documentation](https://microsoft.github.io/AirSim)

[AirSim Installation Instructions for Linux](https://microsoft.github.io/AirSim/build_linux/)

[AirSim ROS Wrapper Documentation](https://microsoft.github.io/AirSim/airsim_ros_pkgs/)

[AirSim Settings Documentation](https://microsoft.github.io/AirSim/settings/)

[Great AirSim Guide Created by Jacopo Panerati Part1](https://github.com/JacopoPan/a-minimalist-guide/blob/master/Part3-Using-AirSim.md)

[Great AirSim Guide Created by Jacopo Panerati Part2](https://github.com/JacopoPan/a-minimalist-guide/blob/master/Part4-Modifying-AirSim.md)
