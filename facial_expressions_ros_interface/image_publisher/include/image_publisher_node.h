#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class ImagePublisherNode {
public:
  
    ros::NodeHandle& nh_;

    explicit ImagePublisherNode(ros::NodeHandle &nh);

    ros::Timer timer_;
    image_transport::Publisher pub_image_;
    image_transport::ImageTransport it;

    cv::Mat input_image;

    void
    CallbackProducer(const ros::TimerEvent&);


};

