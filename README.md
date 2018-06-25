# A trigger for ROS-enabled emergency-stops

This sample code listens to a "heartbeat" topic sent from a ROS-enabled emergency-stop (or a gateway). It will stop the software when emergency stop is pressed and restart it if released.

The heartbeats follow a basic signature and timing scheme that should avoid crosstalk over different emergency stop publishers.

We intend to use this software for [`dynamixel_control_hw`][dynamixel_control_hw] and [`youbot_driver_ros_interface`][youbot_driver_ros_interface] for our robots. On the other end of the line, the original heartbeat is produced by our [Wifi emergency stop button][esp8266-estop] and the corresponding [ROS gateway][gateway] that converts the UDP packets to a ROS topic.

<!-- TODO: drawing of the data flow from e-stop to robot -->

> **Caution**: this implementation of the heartbeat protocol is not complete yet and misses the following features:
>
> - check that the message is not too old
> - check that the counter is strictly increasing (for a given second) and that seconds are ever increasing
>
> Also, the node `ros_control_trigger` is not finished yet, although the main ideas are already there. Please note that this node is a first integration of the emergency-stop protocol for ros_control, based on [the discussion in a git issue][discussion_issue]. It is incorrect and MUST be redesigned later. Indeed, if this node fails, the emergency stop could never make the robot stop. A better approach, although more invasive, would be to integrate with the control loop, that has direct access to the controller manager.
>
> A decision should be made also on how to nicely stop the node if the setup phase fails.

## Parameters of the sample nodes

| name         | description                                                  |
| ------------ | ------------------------------------------------------------ |
| key_path     | path to the file containing the signature key                |
| max_interval | highest allowed time interval between two heartbeat messages |

## Authors

- Author/Maintainer: Dorian Goepp

## Dependencies

- OpenSSL to check the validity of the heartbeat

## Documentation

## LICENSE

[CeCILL-C]

[dynamixel_control_hw]: https://github.com/resibots/dynamixel_control_hw/
[youbot_driver_ros_interface]: https://github.com/youbot/youbot_driver_ros_interface
[CeCILL-C]: http://www.cecill.info/index.en.html
[esp8266-estop]: https://gitlab.inria.fr/dgoepp/esp8266_e_stop
[gateway]: https://gitlab.inria.fr/dgoepp/estop-gateway
[discussion_issue]: https://github.com/ros-controls/ros_control/issues/129
