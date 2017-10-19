#include <ros/ros.h>
#include <estop_gateway_udp/Heartbeat.h>

/**
    Utility : method to search the key based on a ROS parameter name that points to a file

    I need a way to know when the allowed delay is reached. Then, a user-provided function is called.
        Use std something to allow for class methods and more.
        I could use a timer that is constantly reset.

    This class shall also check that an incoming heartbeat has not been accepted before.

    Add the option to notify the user if an invalid message was received
**/
class estopWatchdog {
public:
    estopWatchdog(ros::NodeHandle nh, std::vector<uint8_t> key);
    // arguments : topic name, key

    ~estopWatchdog();

    void heartbeat_callback(const estop_gateway_udp::Heartbeat::ConstPtr& heartbeat);

private:
    estopWatchdog operator=(estopWatchdog&) = 0;
    estopWatchdog(estopWatchdog&) = 0;
}