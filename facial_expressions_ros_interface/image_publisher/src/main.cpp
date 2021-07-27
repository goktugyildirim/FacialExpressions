#include <iostream>
#include <ros/ros.h>
#include "image_publisher_node.h"



int
main(int argc, char** argv) {

    ros::init (argc, argv, "image_publisher_node");
    ros::NodeHandle nh;

    ImagePublisherNode producerConsumerNode(nh);

    ros::spin ();
    ros::waitForShutdown();

    return 0;
}
