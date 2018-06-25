#include <controller_manager_msgs/SwitchController.h>
#include <estop_trigger/estop_trigger.hpp>
#include <estop_trigger/exception.hpp>

#include <ros/ros.h>

bool switch_controllers(bool enable = false)
{
    std::string service = "/controller_manager/switch_controller";

    ros::ServiceClient client
        = nh.serviceClient<controller_manager_msgs::SwitchController>(service);
    controller_manager_msgs::SwitchController srv;

    std::vector<std::string> start_controllers, stop_controllers;
    if (enable)
        start_controllers.push_back("/dynamixel_controllers/joint_state_controller");
    else
        stop_controllers.push_back("/dynamixel_controllers/joint_state_controller");
    srv.request.start_controllers = start_controllers;
    srv.request.stop_controllers = stop_controllers;

    srv.request.strictness = srv.request.BEST_EFFORT;

    if (client.call(srv)) {
        ROS_INFO_STREAM("Service call succeeded. The controllers were"
            << (srv.response.ok ? "" : " NOT") << " switched.");
    }
    else {
        ROS_ERROR("Failed to call the service "
                  "/controller_manager/switch_controllers");
        return false;
    }

    return srv.response.ok;
}

void trip(void)
{
    ROS_INFO("Waf Waf Waf ! J'ai faim ! Oh, mais c'est une belle jambe bien dodue que je vois là !");
    switch_controllers(false);

}
void release(void)
{
    ROS_INFO("Le chien est repus, nous sommes saufs.");
    switch_controllers(true);
}

using namespace estop;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "estop_trigger");
    ros::NodeHandle nh("~");

    EStopTrigger e_trigger(nh, &trip, &release);
    e_trigger.wake();

    std::string service = "/controller_manager/switch_controller";
    if (ros::service::waitForService(service, 1000)) {
        ROS_INFO("The required service was found");
        ros::ServiceClient client
            = nh.serviceClient<controller_manager_msgs::SwitchController>(service);
        controller_manager_msgs::SwitchController srv;
        std::vector<std::string> start_controllers, stop_controllers;
        start_controllers.push_back("/dynamixel_controllers/joint_state_controller");
        // stop_controllers.push_back("/dynamixel_controllers/joint_state_controller");
        srv.request.strictness = srv.request.BEST_EFFORT;
        srv.request.start_controllers = start_controllers;
        srv.request.stop_controllers = stop_controllers;
        if (client.call(srv)) {
            ROS_INFO_STREAM("Service call succeeded. The controllers were"
                << (srv.response.ok ? "" : " NOT") << " switched.");
        }
        else {
            ROS_ERROR("Failed to call the service "
                      "/controller_manager/switch_controllers");
            return 1;
        }
    }
    else
        ROS_ERROR("The required service is not available");

    ros::spin();

    return 0;
}