#include <iostream>
#include <cstring>
#include <utility> // for std::pair
// #include <algorithm>
// #include <iterator>

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

bool equal(const unsigned char* begin1, const unsigned char* begin2, const unsigned int length)
{
    for (unsigned int i = 0; i < length; ++i) {
        if (begin1[i] != begin2[i])
            return false;
    }
    return true;
}

void new_heartbeat(const estop_gateway_udp::Heartbeat::ConstPtr& heartbeat)
{
    std::cout << "New heartbeat" << std::endl;
    char key[25];
    strcpy(key, "08:36:55");

    size_t key_length = strlen(key);

    // Compute the hash of the message
    const uint32_t seconds = heartbeat->header.stamp.sec;
    std::cout << std::dec << "\tSeconds: " << (int)seconds << std::endl;
    const uint16_t count = heartbeat->count;
    std::cout << "\tCount: " << (int)count << std::endl;

    static constexpr size_t message_len = sizeof(seconds) + sizeof(count);
    unsigned char message[message_len];
    memcpy(message, &seconds, sizeof(seconds));
    memcpy(message + sizeof(seconds), &count, sizeof(count));

    std::cout << "\tPayload" << std::endl
              << "\t\t";
    print_hex(message, message_len);

    // std::cout << "\tSeconds: " << (int)*((uint32_t*)message) << std::endl;
    // std::cout << "\tCount: " << (int)*((uint8_t*)message + sizeof(seconds)) << std::endl;

    // Based on the documentation at: https://www.openssl.org/docs/man1.0.2/crypto/HMAC_CTX_init.html
    unsigned char result[EVP_MAX_MD_SIZE];
    unsigned int result_length;
    HMAC(EVP_sha256(), (void*)key, strlen(key), (const unsigned char*)message, message_len, result, &result_length);

    // Authentication provided in the message

    const unsigned char* reference_hash;
    // convert std::vector<uint8_t> to array of char
    reference_hash = &heartbeat->hash[0];

    // Compare locally_computed hash to the one we received i.e. authenticate the message

    std::cout << "\tLocal hash" << std::endl
              << "\t\t";
    print_hex(result, result_length);

    std::cout << "\tReceived hash" << std::endl
              << "\t\t";
    print_hex(reference_hash, result_length);

    // std::equal(std::begin(result), std::end(result), std::begin((const unsigned char[])reference_hash))
    if (equal(result, reference_hash, result_length))
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

    // char key[25], message[25];
    // strcpy(key, "08:36:55");
    // strcpy(message, "message");
    //
    // unsigned char result[EVP_MAX_MD_SIZE];
    // unsigned int result_length;
    // // Based on the documentation at: https://www.openssl.org/docs/man1.0.2/crypto/HMAC_CTX_init.html
    // HMAC(EVP_sha256(), (void*)key, strlen(key), (const unsigned char*)message, strlen(message), result, &result_length);
    //
    // std::cout << "Local hash" << std::endl;
    // for (unsigned int i = 0; i < result_length; ++i) {
    //     if (i > 0)
    //         std::cout << ':';
    //     std::cout << std::hex << (int)result[i];
    // }
    // std::cout << std::endl;

    return 0;
}

// class HmacSha256 {
// public:
//     HmacSha256(const void* key, int key_len) : _key(), _key_len(key_len)
//     {
//         HMAC_CTX_init(&_ctx);
//     }
//
//     ~HmacSha256()
//     {
//         HMAC_CTX_cleanup(&_ctx);
//     }
//
//     std::pair<unsigned char*, unsigned int> compute(unsigned char* message, int message_len)
//     {
//         unsigned char result[EVP_MAX_MD_SIZE];
//         unsigned int result_length;
//
//         HMAC(EVP_sha256(), (void*)key, strlen(key), (const unsigned char*)message, strlen(message), result, &result_length);
//
//         return std::make_pair(result, result_length);
//     }
//
//     std::pair<unsigned char*, unsigned int> compute_str(unsigned char* message)
//     {
//         return compute(message, strlen(message));
//     }
//
// private:
//     const void* _key;
//     const int _key_len;
//     HMAC_CTX _ctx;
// };