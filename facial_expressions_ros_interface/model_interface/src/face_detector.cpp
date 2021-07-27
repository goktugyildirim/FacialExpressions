//
// Created by goktug on 25.07.2021.
//

#include "face_detector.h"


void
FaceDetector::detectAndDraw(cv::Mat& img, cv::CascadeClassifier& cascade,
              double scale,
              std::vector<std::pair<size_t, cv::Mat>>& vector_face_and_id)
{
    std::vector<cv::Rect> faces;
    cv::Mat gray, smallImg;

    cvtColor( img, gray, cv::COLOR_BGR2GRAY ); // Convert to Gray Scale
    double fx = 1 / scale;

    // Resize the Grayscale Image
    resize( gray, smallImg, cv::Size(), fx, fx, cv::INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    // Detect faces of different sizes using cascade classifier
    cascade.detectMultiScale( smallImg, faces, 1.1,
                              2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );


    // Draw rect around the faces
    for ( size_t i = 0; i < faces.size(); i++ )
    {
        cv::Rect r = faces[i];
        cv::Mat smallImgROI;
        std::vector<cv::Rect> nestedObjects;
        cv::Point center;
        cv::Scalar color = cv::Scalar(255, 0, 0); // Color for Drawing tool

        rectangle(img, cv::Point(cvRound(r.x*scale), cvRound(r.y*scale)),
                  cv::Point(cvRound((r.x + r.width-1)*scale),
                            cvRound((r.y + r.height-1)*scale)), color, 3, 8, 0);

        cv::Rect r_modified(cv::Point(cvRound(r.x*scale),
        cvRound(r.y*scale)),
        cv::Point(cvRound((r.x + r.width-1)*scale),
                  cvRound((r.y + r.height-1)*scale)));

        // Show Processed Image with detected faces
        smallImgROI = img(r_modified);
        std::pair<size_t, cv::Mat> face_and_id;
        face_and_id.first = i;
        face_and_id.second = smallImgROI;

        vector_face_and_id.push_back(face_and_id);
        std::string name = "/home/goktug/Desktop/detection_result_" + std::to_string(i) + ".jpg";
        cv::imwrite(name, smallImgROI);
    }
}
