//
// Created by goktug on 25.07.2021.
//


#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>




class FaceDetector
{
public:

    static void
    detectAndDraw(cv::Mat& img, cv::CascadeClassifier& cascade,
                  double scale,
                  std::vector<std::pair<size_t, cv::Mat>>& vector_face_and_id);



};


