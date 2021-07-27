#include <iostream>
#include <ros/ros.h>
#include "model_interface_node.h"

int
main(int argc, char** argv) {

    ros::init (argc, argv, "model_interface_node");
    ros::NodeHandle nh;

    ModelInterfaceNode modelInterfaceNode(nh);

    ros::spin ();
    ros::waitForShutdown();

    return 0;
}
