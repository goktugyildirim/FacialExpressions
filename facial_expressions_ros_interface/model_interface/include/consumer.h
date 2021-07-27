//
// Created by goktug on 26.07.2021.
//

#include "face.h"

#include <iostream>
#include <memory>
#include <mutex>
#include <deque>
#include <future>
#include <ros/ros.h>
#include <atomic>
#include "face_classifier.h"
class Consumer
{
public:
    using Ptr = std::shared_ptr<Consumer>;
    int consumer_id;
    explicit Consumer(const int& consumer_id_);

    FaceClassifier faceClassifier;

    std::shared_future<void> future_;

    void StartConsumer(std::deque<Face> &stack, std::mutex &door, std::atomic<int>& processed_face_count, std::vector<Face>& vector_faces_output);
    void Run(std::deque<Face>& stack, std::mutex& door, std::atomic<int>& processed_face_count, std::vector<Face>& vector_faces_output);
};


