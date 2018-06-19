#include <ros/ros.h>
// #include <ros/forwards.h>
#include <estop_gateway_udp/Heartbeat.h>

namespace estop {
    /** Utility : method to search the key based on a ROS parameter name that points to a file

        I need a way to know when the allowed delay is reached. Then, a user-provided function is called.
            Use std something to allow for class methods and more.
            I could use a timer that is constantly reset.

        This class shall also check that an incoming heartbeat has not been accepted before.

        Add the option to notify the user if an invalid message was received
    **/
    class estopTrigger {
    public:
        estopTrigger(ros::NodeHandle nh,
            ros::Duration max_interval,
            std::function<void()> stop_callback,
            std::function<void()> resume_callback,
            std::vector<uint8_t> key);
        // arguments : topic name, key

        ~estopTrigger();

        void wake();
        void sleep();

        // TODO: something to notify the user that the heartbeat resumed (after they stopped behond max_interval)
        // TODO: better callback interface, not requiring to accept a TimerEvent (unless relevant)

    private:
        // Don't define these functions, we don't want them to exist
        estopTrigger& operator=(estopTrigger&);
        estopTrigger(estopTrigger&);

        void heartbeat_callback(const estop_gateway_udp::Heartbeat::ConstPtr& heartbeat);
        // Feed the trigger, restarting the timer.
        void feed();
        void timeout_callback(const ros::TimerEvent&);

        bool check_heartbeat(const estop_gateway_udp::Heartbeat::ConstPtr& heartbeat);

        ros::NodeHandle _nh;
        ros::Subscriber _heartbeat_subscriber;
        std::function<void()> _stop_callback;
        std::function<void()> _resume_callback;
        ros::Timer _timer;
        bool _timeout;

        std::vector<uint8_t> _key;
    };
} // namespace estop