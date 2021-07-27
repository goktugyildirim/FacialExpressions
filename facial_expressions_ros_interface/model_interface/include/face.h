//
// Created by goktug on 25.07.2021.
//

#include <memory>
#include <opencv2/highgui/highgui.hpp>

class Face{
public:
    using Ptr = std::shared_ptr<Face>;
    cv::Mat face_img;
    std::string class_name;
    bool processed = false;
    int face_id = -1;
    int consumer_id = -1;
    double prob;
};

