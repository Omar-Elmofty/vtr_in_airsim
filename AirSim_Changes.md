
# AirSim Changes

This document contains all the changes made to airsim's master branch available at `https://github.com/microsoft/AirSim.git`.
The modified code is in the below forked repository:

```
https://github.com/Omar-Elmofty/AirSim.git
```

### Change to AirLib

AirLib is the library containing all the drone modeling code. The library contains several firmwares (SimpleFlight, AruCopter & MavLink). ArduCopter and MavLink are meant for hardware in the loop testing with flight controllers. All the changes were made to the SimpleFlight firmware which originally made to fully simulate a quadcopter, hence the changes were mainly made so SimpleFlight can accept a HexaCopter model.



### `SimpleQuadXParams.hpp` in `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight`:


Changed 

Changed a lot of the parameters


`AirSimSimpleFlightBoard.hpp` under `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight`

Line 70 (function getAvgMotorOutput), added two more motors (6 total)

`Mixer.hpp` under `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight/firmware`

Changed mixer matrix to include 6 motor outputs





`Params.hpp` under `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight/firmware`

Change uint16_t motor_count = 6;


`FlyingPawn.h` under`~/AirSim/Unreal/Plugins/AirSim/Source/Vehicles/Multirotor`







Add Z-rate function

`~/AirSim_Forked/AirLib/src/vehicles/multirotor/api` all files under there

`~/AirSim_Forked/AirLib/include/vehicles/multirotor/api` - API base, and RPClib client

`~/AirSim_Forked/AirLib/include/vehicles/multirotor/firmwares/mavlink/MavLinkMultirotorApi.hpp`
`~/AirSim_Forked/AirLib/include/vehicles/multirotor/firmwares/arducopter/ArduCopterApi.hpp` 
`~/AirSim_Forked/AirLib/include/vehicles/multirotor/firmwares/simple_flight/SimpleFlightApi.hpp`




Also CMakeLists for ROS Wrapper


client.py for python client



