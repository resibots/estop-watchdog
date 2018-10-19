# A trigger for ROS-enabled emergency-stops

This sample code listens to a "heartbeat" topic sent from a ROS-enabled emergency-stop (or a gateway to ROS-incompatible one). It uses two callbacks to stop something when the emergency stop is pressed and restart it if released.

The heartbeats follow a basic signature and timing scheme that should avoid crosstalk over different emergency stop publishers.

We intend to use this software for [`dynamixel_control_hw`][dynamixel_control_hw] and [`youbot_driver_ros_interface`][youbot_driver_ros_interface] for our robots. On the other end of the line, we have our [Wifi emergency stop button][esp8266-estop] produce the original heartbeat and a [ROS gateway][gateway] converts the UDP packets to a ROS topic.

<!-- TODO: drawing of the data flow from e-stop to robot -->

> **Caution**: this implementation of the heartbeat protocol is not complete yet and misses the following features:
>
> - check that the message is not too old
> - check that the counter is strictly increasing (for a given second) and that seconds are ever increasing
>
> Also, the node `ros_control_trigger` is not finished yet, although the main ideas are already there. Please note that this node is a first integration of the emergency-stop protocol for ros_control, based on [the discussion in a git issue][discussion_issue]. It is incorrect and MUST be redesigned later. Indeed, if this node fails, the emergency stop could never make the robot stop. A better approach, although more invasive, would be to integrate with the control loop, that has direct access to the controller manager.
>
> A decision should be made also on how to nicely stop the node if the setup phase fails.

## Assumptions

Some of the assumptions that we make for this system to work well:

- no one will fuss with the ros parameters, for instance increasing the `max_interval` parameter to an unreasonably high value
- the key file is not reachable by undesired parties

## Parameters of the sample nodes

| name         | description                                                  |
| ------------ | ------------------------------------------------------------ |
| key_path     | path to the file containing the signature key                |
| max_interval | highest allowed time interval between two heartbeat messages (in ms)|

## Usage

A sample program, `estop_trigger_sample` is provided for the user to see how this code is meant to be used. It can be run with the provided launch file and configuration. Remember, however, to put the actual path to the key in the configuration file (`config/sample.yaml`).

## Future improvements

### Reset the controllers when resuming

When the emergency button is released and the hardware interface would be enabled again, we should have the `update` method of the controller manager be called with the argument `reset_controller` set to true, so that all controllers are stopped and started. This should avoid unwanted behaviour, such as the robot continuing on the course that made us trigger the emergency stop.

### Integrate directly with the hardware interfaces

We would like to have the hardware interfaces be directly told to stop the actuators and ignore the commands sent by the controllers. Actually, we would like to let the hardware interface decide of what policy to be enforced when the emergency button is pressed. To do so, we beleive that there should be two methods added in the hardware interface and combined hardware interface, in ros control. For more on the matter, se [this pull request on Github](https://github.com/ros-controls/ros_control/pull/294).

## Authors

- Author/Maintainer: Dorian Goepp

## Dependencies

- OpenSSL to check the validity of the heartbeat

## LICENSE

Unless stated otherwise, this work is published under the terms of the [CeCILL-C] license.

[dynamixel_control_hw]: https://github.com/resibots/dynamixel_control_hw/
[youbot_driver_ros_interface]: https://github.com/youbot/youbot_driver_ros_interface
[CeCILL-C]: http://www.cecill.info/index.en.html
[esp8266-estop]: https://gitlab.inria.fr/dgoepp/esp8266_e_stop
[gateway]: https://gitlab.inria.fr/dgoepp/estop-gateway
[discussion_issue]: https://github.com/ros-controls/ros_control/issues/129
