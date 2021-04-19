# spar_node
The driving node for a mid-level navigation software to implement flight actions with PX4 and ROS.

## Installing Dependencies
Please download and install using the instructions for the [QUT Flight Stack](https://github.com/qutas/info/wiki/UAV-Setup-Guides-(2021)#the-qutas-flight-stack)

## Running

```sh
roslaunch spar_node spar.launch
```

#### Other Launch Files
- If you have the `mavros` packages installed, you can use the following launch file to connect to a PX4-based autopilot: `spar_mavros.launch`
- If you have the `uavasr_emulator` package installed, you can use the following launch file to start `spar` and the [UAVASR emulator](https://www.github.com/qutas/uavasr_emulator): `spar_uavasr.launch`
- If you have the `uavusr_emulator` package installed, you can use the following launch file to start `spar` and the [UAVUSR emulator](https://www.github.com/qutas/uavusr_emulator): `spar_uavusr.launch`

#### Usage
The `spar_node` will present the `~/flight` [action](https://www.github.com/qutas/spar/blob/main/spar_msgs/action/FlightMotion.action) once the following conditions are met:
- The UAV is armed
- The UAV is in `OFFBOARD` mode

The flight action will stay active as long as these conditions hold. If the UAV changes mode or disarms, the any current actions will be cancelled. In this way, the action is only presented in the situation where the UAV is ready for flight. This means that the `wait_for_server()` function in the `simple_action_client` can be used to wait for the action to become available (which in turn allows any other navigation nodes to be started in any order).

#### Helpful, Demo, & Example Code
Please see the `scripts` folder for integration examples of the `~/flight` action:
- `takeoff`: Performs a take-off at a configurable speed to a set altitude at the current location
- `land`: Performs a landing at a configurable speed at the current location (relies on the PX4 auto-disarm function to be marked "complete")
- `takeoff_and_land`: Performs a take-off, short pause, then landing as per the above features
- `goto`: Performs a "Go-To" waypoint; travel from the current locaiton to a set location at a configurabe speed (requires command line arguments, use "--help" option for more information)
- `demo_wp`: Performs a preset 4-waypoint task from with a range of configurable settings (waypoint radius, movement speed, _etc._)
- `demo_wp_roi`: Performs a preset4-waypoint task as above, with additional code to perform a diversion and continuation if a `geometry_msgs/PoseStamped` message is recieved.

## Subscribed Topics
- Autopilot State ([mavros\_msgs/State](http://docs.ros.org/api/mavros_msgs/html/msg/State.html)): `/spar/mav_state`
- Current Position ([geometry\_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)): `/spar/pose`

## Published Topics
- Local Position Setpoint ([mavros\_msgs/PositionTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html)): `/spar/setpoint`
