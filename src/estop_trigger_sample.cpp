#include<estop_trigger/estop_trigger.hpp>

#include<ros/ros.h>

void alerte(void)
{
    ROS_WARN("Waf Waf Waf ! J'ai faim ! Oh, mais c'est une belle jambe bien dodue que je vois là !");
}

void ouf(void)
{
    ROS_INFO("Le chien est repus, nous sommes saufs.");
}

using namespace estop;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "estop_trigger");
    ros::NodeHandle nh("~");

    // Instantiate the trigger
    EStopTrigger e_trigger(nh, &alerte, &ouf);
    // Enable it (since it does not automatically start)
    e_trigger.wake();

    ros::spin();

    return 0;
}