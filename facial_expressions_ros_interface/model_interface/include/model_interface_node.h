#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "face_detector.h"
#include <deque>
#include <mutex>
#include "consumer.h"
#include <atomic>


// Parallel programming library:
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/sequence.h>
#include <thrust/execution_policy.h>

class ModelInterfaceNode {
public:
  
    ros::NodeHandle& nh_;

    image_transport::ImageTransport it;
    image_transport::Subscriber sub;

    explicit ModelInterfaceNode(ros::NodeHandle &nh);

    void
    ConsumerCallback(const sensor_msgs::ImageConstPtr& msg);



    // Face detection:
    cv::CascadeClassifier cascade_face_detection_model;
    double scale=3;

    std::deque<Face> stack_faces;
    std::vector<std::pair<size_t, cv::Mat>> vector_face_and_id;
    std::vector<Face> vector_faces_output;

    Consumer::Ptr consumer_1_;
    Consumer::Ptr consumer_2_;
    Consumer::Ptr consumer_3_;
    Consumer::Ptr consumer_4_;
    Consumer::Ptr consumer_5_;
    Consumer::Ptr consumer_6_;

    std::mutex mutex;
    std::atomic<int> processed_face_count;

    image_transport::Publisher pub_image_;


    static cv::Mat
    PreProcessImage(const cv::Mat& input_image);




};

