#include "image_publisher_node.h"

ImagePublisherNode::ImagePublisherNode(ros::NodeHandle &nh)
: nh_(nh), it(nh_)
{
    input_image = imread("/home/goktug/projects/facial_expressions/model/test.jpg", cv::IMREAD_COLOR);
    float hz = 30;
    timer_ = nh_.createTimer(ros::Duration(1/hz),
                             &ImagePublisherNode::CallbackProducer,
                             this);
    pub_image_ = it.advertise("/image", 12);
}

void
ImagePublisherNode::CallbackProducer(const ros::TimerEvent &)
{
    std_msgs::Header  header;
    header.frame_id = "camera_frame";
    header.stamp = ros::Time::now();
    cv_bridge::CvImage cvImage(header, "bgr8", input_image);
    pub_image_.publish(cvImage.toImageMsg());
    std::cout << "Image is published." << std::endl;
}





