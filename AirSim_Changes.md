
# AirSim Changes

This document contains all the changes made to AirSim's master branch available at `https://github.com/microsoft/AirSim.git`.
The modified code is in the below forked repository:

```
https://github.com/Omar-Elmofty/AirSim.git
```

It is highly recommended that you compare the changes made to the original AirSim code base to understand the reasoning behind it. 

## Changes to AirLib

AirLib is the library containing all the drone modeling code. The library contains several firmwares (SimpleFlight, AruCopter & MavLink). ArduCopter and MavLink are meant for hardware in the loop testing with flight controllers. All the changes were made to the SimpleFlight firmware which originally was made to fully simulate a quadcopter, hence the changes were made so SimpleFlight can accept a HexaCopter model.


#### `SimpleQuadXParams.hpp` 

Located in `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight`


Changes are in `setupParams` function. Updated all the drone parameters to values that match the M600 (such as rotor_count, mass, arm_lengths .. etc), also used `initializeRotorHexX` rather than `initializeRotorQuadX` for initialization of rotors.


####`Params.hpp` 

Located in `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight/firmware`

Changed `motor_count`  in `struct Motor` to be 6 rather than 4


#### `AirSimSimpleFlightBoard.hpp`

Located in `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight`

Line 70 (function getAvgMotorOutput), added two more motors (6 total).


#### `Mixer.hpp` 

Located in `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight/firmware`

Changed the mixer matrix `mixerQuadX` to `mixerHexX`, increased its size to 6 and adjusted its values to mix the outputs of 6 motors rather than 4.

## Other Changes outside AirLib

#### `FlyingPawn.h`

Located in `~/AirSim/Unreal/Plugins/AirSim/Source/Vehicles/Multirotor`

Changed `rotor_count` private variable to 6 rather than 4. This will help in rendering 6 motors rather than 4 in AirSim.


## Changes to AirSim's API

In order to run AirSim with VT&R, AirSim must accept inputs in the form [roll, pitch, yaw_rate, z_rate]. There was no function readily available in the API to accept this set of inputs, hence a new function was created.


#### `client.py`

Located in `~/AirSim/PythonClient/airsim`

Added `moveByRollPitchYawrateZrateAsync` function. 

#### `MultirotorApiBase.cpp`

Located in `~/AirSim/AirLib/src/vehicles/multirotor/api`

Added functions `moveByRollPitchYawrateZrate` and `moveByRollPitchYawrateZrateInternal`

#### `MultirotorRpcLibClient.cpp`

Located in `~/AirSim/AirLib/src/vehicles/multirotor/api`

Added Function `moveByRollPitchYawrateZrateAsync`

#### `MultirotorRpcLibServer.cpp`

Located in `~/AirSim/AirLib/src/vehicles/multirotor/api`

Added Bind command for `moveByRollPitchYawrateZrate` function

#### `SimpleFlightApi.hpp`

Located in `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight`

Added `commandRollPitchYawrateZrate`, which has the `GoalMode` variable set to the required inputs [roll, pitch, yaw_rate, z_rate].

#### `MavLinkMultirotorApi.hpp`

Located in `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/mavlink`

Added `commandRollPitchYawrateZrate` Note that this function is incomplete, however it was just added to avoid errors during compiling.

#### `ArduCopterApi.hpp`

Located in `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/arducopter` 

Added `commandRollPitchYawrateZrate` function. 

