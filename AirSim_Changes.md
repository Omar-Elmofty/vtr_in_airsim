


`AirSimSimpleFlightBoard.hpp` under `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight`

Line 70 (function getAvgMotorOutput), added two more motors (6 total)

`Mixer.hpp` under `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight/firmware`

Changed mixer matrix to include 6 motor outputs


`SimpleQuadXParams.hpp` under `~/AirSim/AirLib/include/vehicles/multirotor/firmwares/simple_flight`

Changed a lot of the parameters



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



