//
// Created by goktug on 26.07.2021.
//

#include "consumer.h"

Consumer::Consumer(const int& consumer_id_)
        :consumer_id(consumer_id_),
        faceClassifier("/home/goktug/projects/facial_expressions/model/facial_expression_model.onnx")
{
    std::cout << "Consumer " << consumer_id << " is initialized!" << std::endl;
}


void
Consumer::Run(std::deque<Face> &stack,
              std::mutex &door,
              std::atomic<int>& processed_face_count,
              std::vector<Face>& vector_faces_output)
{
    Face face_classified;

    while (ros::ok())
    {
        cv::Mat face_to_be_classified;
        int face_id;
        bool process_required = false;

        door.lock();
        if (!stack.empty())
        {
            process_required = true;
            face_id = stack.front().face_id;
            face_to_be_classified = stack.front().face_img.clone();
            //std::cout << "Running " << consumer_id << ". consumer. Face id: " << stack.front().face_id << std::endl;
            stack.pop_front();
        }
        door.unlock();


        if (process_required)
        {
            auto result = faceClassifier.Classify(face_to_be_classified);
            face_classified.face_id = face_id;
            face_classified.class_name = result.first;
            face_classified.prob = result.second;
            face_classified.processed = true;
            face_classified.consumer_id = consumer_id;

            vector_faces_output.push_back(face_classified);
            // condition variable with the type of atomic, no requiring mutex lock
            processed_face_count += 1;
        }


    }

}

void
Consumer::StartConsumer(std::deque<Face> &stack,
                        std::mutex &door,
                        std::atomic<int>& processed_face_count,
                        std::vector<Face>& vector_faces_output)
{
    future_ = std::async(std::launch::async,
                         &Consumer::Run,
                         this,
                         std::ref(stack),
                         std::ref(door),
                         std::ref(processed_face_count),
                         std::ref(vector_faces_output));
}

