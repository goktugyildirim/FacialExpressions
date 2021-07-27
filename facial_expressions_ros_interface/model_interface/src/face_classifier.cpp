//
// Created by goktug on 26.07.2021.
//

#include "face_classifier.h"


FaceClassifier::FaceClassifier(const std::string &path_model)
{
    // Read ONNX model to inference with OpenCV DNN module:
    classifier = cv::dnn::readNetFromONNX(path_model);
    classifier.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    classifier.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    output_names = classifier.getUnconnectedOutLayersNames();
}


std::pair<std::string, double>
FaceClassifier::Softmax(const std::vector<cv::Mat>& input,
        const std::vector<std::string>& classes)
{
    std::vector<double> softmax_output;

    /*for(size_t i=0; i<5; i++){
        std::cout << input[0].at<float>(0,i)<< std::endl;
    }*/

    //std::cout << input[0] << std::endl;

    softmax_output.resize(5);
    float sum;
    for(size_t i=0; i<5; i++){
        sum = sum + exp(input[0].at<float>(0,i));
        softmax_output[i] = exp(input[0].at<float>(0,i));
    }
    for(size_t i=0; i<5; i++){
        softmax_output[i] = softmax_output[i]/sum;
        //std::cout << softmax_output[i] << std::endl;
    }

    auto max_id = std::distance(softmax_output.begin(),
                                std::max_element(softmax_output.begin(),
                                softmax_output.end()));

    std::pair<std::string, double> pair;
    pair.first = classes[max_id];
    pair.second = softmax_output[max_id];
    return pair;
}

cv::Mat
FaceClassifier::PreProcessImage(const cv::Mat& input_image)
{
    const cv::Mat& imageBGR = input_image;
    cv::Mat resizedImageBGR, resizedImageRGB, resizedImage, preprocessedImage;
    cv::resize(imageBGR, resizedImageBGR,
               cv::Size(224, 224),
               cv::InterpolationFlags::INTER_CUBIC);

    // Change channel order because ResNet model is trained using RGB images:
    cv::cvtColor(resizedImageBGR, resizedImageRGB,
                 cv::ColorConversionCodes::COLOR_BGR2RGB);
    // Standardization:
    resizedImageRGB.convertTo(resizedImage, CV_32F, 1.0 / 255);

    cv::Mat channels[3];
    cv::split(resizedImage, channels);
    // Normalization per channel
    // Normalization parameters obtained from
    // https://github.com/onnx/models/tree/master/vision/classification/squeezenet
    channels[0] = (channels[0] - 0.485) / 0.229;
    channels[1] = (channels[1] - 0.456) / 0.224;
    channels[2] = (channels[2] - 0.406) / 0.225;
    cv::merge(channels, 3, resizedImage);
    // HWC to CHW
    cv::dnn::blobFromImage(resizedImage, preprocessedImage);
    return preprocessedImage;
}


std::pair<std::string, double>
FaceClassifier::Classify(const cv::Mat &image)
{
    //cv::Mat preprocessed_image = PreProcessImage(image);
    classifier.setInput(image);
    std::vector<cv::Mat> output_blobs;
    classifier.forward(output_blobs, output_names);
    std::pair<std::string, double>  class_result = Softmax(output_blobs, class_names);
    return class_result;
}

