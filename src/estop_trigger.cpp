#include <estop_gateway_udp/Heartbeat.h>
#include <estop_trigger/estop_trigger.hpp>
#include <estop_trigger/exception.hpp>
#include <ros/ros.h>

#include <openssl/hmac.h>

void print_hex(const unsigned char* array, const unsigned int array_length)
{
    for (unsigned int i = 0; i < array_length; ++i) {
        if (i > 0)
            std::cout << ':';
        std::cout << std::hex << (int)array[i];
    }
    std::cout << std::endl
              << std::dec;
}

namespace estop {

    EStopTrigger::EStopTrigger(ros::NodeHandle nh,
        ros::Duration max_interval,
        std::function<void()> stop_callback,
        std::function<void()> resume_callback,
        std::vector<uint8_t> key)
        : _nh(nh), _key(key), _stop_callback(stop_callback), _resume_callback(resume_callback)
    {
        // An empty key should not be accepted
        if (0 == _key.size())
            throw Exception("The key provided is empty (size 0), there must be a mistake.");

        // boolean options : enable oneshot, disable autostart
        // TODO: once we move on from Kinetic Kame, use the line below
        // _timer = nh.createSteadyTimer(max_interval, callback, true, false);
        _timer = nh.createTimer(max_interval, &EStopTrigger::timeout_callback, this, true, false);
        // TODO: decide the action to do in this case. The object cannot be used.
        if (!_timer) {
            ROS_ERROR_STREAM("We failed creating the trigger timer.");
            throw Exception("We failed creating the trigger timer.");
        }

        _heartbeat_subscriber = _nh.subscribe("heartbeat", 1, &EStopTrigger::heartbeat_callback, this);
        // TODO: decide the action to do in this case. The object cannot be used.
        if (!_heartbeat_subscriber) {
            ROS_ERROR_STREAM("We failed subscribing to the heartbeat topic.");
            throw Exception("We failed subscribing to the heartbeat topic.");
        }
    }

    EStopTrigger::~EStopTrigger()
    {
        _timer.stop();
    }

    void EStopTrigger::sleep()
    {
        _timer.stop();
    }

    void EStopTrigger::wake()
    {
        _timer.start();
    }

    void EStopTrigger::heartbeat_callback(const estop_gateway_udp::Heartbeat::ConstPtr& heartbeat)
    {
        if (check_heartbeat(heartbeat))
            feed();
        else
            ROS_DEBUG("An invalid heartbeat just arrived.");
    }

    void EStopTrigger::feed()
    {
        _timer.stop();
        _timer.start();
        if (_timeout) {
            _timeout = false;
            _resume_callback();
        }
        ROS_DEBUG("Feeding the dog");
    }

    void EStopTrigger::timeout_callback(const ros::TimerEvent&)
    {
        _timeout = true;
        _stop_callback();
    }

    bool EStopTrigger::check_heartbeat(const estop_gateway_udp::Heartbeat::ConstPtr& heartbeat)
    {
        // ROS_DEBUG("Heartbeat received");

        // Compute the hash of the message

        const uint32_t seconds = heartbeat->header.stamp.sec;
        const uint16_t count = heartbeat->count;

        static constexpr size_t payload_len = sizeof(seconds) + sizeof(count);
        unsigned char payload[payload_len];
        memcpy(payload, &seconds, sizeof(seconds));
        memcpy(payload + sizeof(seconds), &count, sizeof(count));

        // from documentation:
        // https://www.openssl.org/docs/man1.0.2/crypto/HMAC_CTX_init.html
        unsigned char result[EVP_MAX_MD_SIZE];
        unsigned int result_length = 0;
        unsigned char* hmac_status
            = HMAC(EVP_sha256(),
                (void*)_key.data(), _key.size(),
                (const unsigned char*)payload, payload_len,
                result, &result_length);

        if (NULL == hmac_status) {
            ROS_ERROR("HMAC computation failed. "); // TODO: better message
            return false;
        }
        else if (0 == result_length) {
            ROS_ERROR("HMAC computation failed : the hashing result is empty.");
            return false;
        }

        // Get the authentication embedded in the message

        const unsigned char* reference_hash;
        // convert std::vector<uint8_t> to array of char
        reference_hash = &heartbeat->hash[0];

        // Compare locally_computed hash to the one we received i.e. authenticate the message
        return (memcmp(result, reference_hash, result_length) == 0);

        // FIXME: check that the message is not too old
        // FIXME: check that the counter is ever increasing (for a given second)
        //  and that seconds are ever increasing
    }
} // namespace estop

void alerte(void)
{
    std::cout << "Waf Waf Waf ! J'ai faim ! Oh, mais c'est une belle jambe bien dodue que je vois là !" << std::endl;
}

void ouf(void)
{
    std::cout << "Le chien est repus, nous sommes saufs." << std::endl;
}

using namespace estop;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "estop_trigger");
    ros::NodeHandle nh("~");
    ros::Duration max_interval(0.50); // five seconds
    // TODO: read key from file, which path is provided by a parameter
    std::vector<uint8_t> key = {48, 56, 58, 51, 54, 58, 53, 53}; // "08:36:55"

    EStopTrigger etrigger(nh,
        max_interval,
        &alerte,
        &ouf,
        key);

    etrigger.wake();

    ros::spin();

    return 0;
}