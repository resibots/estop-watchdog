#include <iostream>
#include <cstring>

#include <openssl/hmac.h>

#include <ros/ros.h>
// #include <ros/time.h>
#include <estop_gateway_udp/Heartbeat.h>

static constexpr size_t HASH_SIZE = 32; // Size in bytes of the hashes we use (sha256)

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

void new_heartbeat(const estop_gateway_udp::Heartbeat::ConstPtr& heartbeat)
{
    std::cout << "New heartbeat" << std::endl;
    char key[25];
    strcpy(key, "08:36:55");

    size_t key_length = strlen(key);

    // Compute the hash of the message
    const uint32_t seconds = heartbeat->header.stamp.sec;
    const uint16_t count = heartbeat->count;

    static constexpr size_t message_len = sizeof(seconds) + sizeof(count);
    unsigned char message[message_len];
    memcpy(message, &seconds, sizeof(seconds));
    memcpy(message + sizeof(seconds), &count, sizeof(count));

    // Based on the documentation at: https://www.openssl.org/docs/man1.0.2/crypto/HMAC_CTX_init.html
    unsigned char result[EVP_MAX_MD_SIZE];
    unsigned int result_length;
    HMAC(EVP_sha256(), (void*)key, strlen(key), (const unsigned char*)message, message_len, result, &result_length);

    // Authentication provided in the message

    const unsigned char* reference_hash;
    // convert std::vector<uint8_t> to array of char
    reference_hash = &heartbeat->hash[0];

    // Compare locally_computed hash to the one we received i.e. authenticate the message

    if (memcmp(result, reference_hash, result_length) == 0)
        std::cout << "\tHash Match !" << std::endl;
    else
        std::cout << "\tHash Mismatch !" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_control_hw");
    ros::NodeHandle nh;

    ros::NodeHandle nhParams("~");
    bool got_all_params = true;

    // std::string usb_serial_interface;
    // got_all_params &= nhParams.getParam("serial_interface", usb_serial_interface);

    ros::Subscriber heartbeat = nh.subscribe("/estop_heartbeat_udp/heartbeat", 0, new_heartbeat);

    ros::spin();

    return 0;
}