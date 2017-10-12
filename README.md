# A watchdog for ROS-enabled emergency-stops
This sample code listens to a "pulse" topic sent from a ROS-enabled emergency-stop (or a gateway) to stop the software when emergency stop is pressed and restart it if released.

TODO: There should be an option for the "acknolwedge" to be required

We use this software in `dynamixel_control_hw` and `youbot_driver_ros_interface` for our robots. On the other end of the line, we use our [Wifi emergency stop][esp8266-estop] and the corresponding ROS gateway.

TODO: drawing of the dataflow from e-stop to robot

## Authors

- Author/Maintainer: Dorian Goepp

## Dependencies

- OpenSSL to check the validity of the pulse

## Documentation


## LICENSE

[CeCILL-C]

[CeCILL-C]: http://www.cecill.info/index.en.html
[esp8266-estop]: https://gitlab.inria.fr/dgoepp/esp8266_e_stop
[gateway]: https://gitlab.inria.fr/dgoepp/estop-gateway