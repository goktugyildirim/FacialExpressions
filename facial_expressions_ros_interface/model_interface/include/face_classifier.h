//
// Created by goktug on 26.07.2021.
//

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

class FaceClassifier
{
public:
    explicit FaceClassifier(const std::string& path_model);

    cv::dnn::Net classifier;
    std::vector<std::basic_string<char>> output_names;

    std::pair<std::string, double>
    Softmax(const std::vector<cv::Mat>& input,
            const std::vector<std::string>& classes);

    cv::Mat
    PreProcessImage(const cv::Mat& input_image);

    std::pair<std::string, double>
    Classify(const cv::Mat& image);

    // Classes in the dataset:
    std::vector<std::string> class_names = {"Eye Closed",
                                            "Neutral",
                                            "Shocked",
                                            "Smile",
                                            "Sunglass"};


};


