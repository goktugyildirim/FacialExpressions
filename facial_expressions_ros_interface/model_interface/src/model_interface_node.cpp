#include "model_interface_node.h"

#include <chrono>
using namespace std::chrono;

ModelInterfaceNode::ModelInterfaceNode(ros::NodeHandle &nh)
: nh_(nh), it(nh_), processed_face_count(0)
{
    // Load face detection model:
    cascade_face_detection_model.load( "/home/goktug/projects/facial_expressions/model/haarcascade_frontalface_default.xml" );

    // Init and start consumer threads to classify faces:
    consumer_1_ = std::make_shared<Consumer>(1);
    consumer_1_->StartConsumer(stack_faces,
                               mutex,
                               processed_face_count,
                               vector_faces_output);



    consumer_2_ = std::make_shared<Consumer>(2);
    consumer_2_->StartConsumer(stack_faces,
                               mutex,
                               processed_face_count,
                               vector_faces_output);

    consumer_3_ = std::make_shared<Consumer>(3);
    consumer_3_->StartConsumer(stack_faces,
                               mutex,
                               processed_face_count,
                               vector_faces_output);

    consumer_4_ = std::make_shared<Consumer>(4);
    consumer_4_->StartConsumer(stack_faces,
                               mutex,
                               processed_face_count,
                               vector_faces_output);

    consumer_5_ = std::make_shared<Consumer>(5);
    consumer_5_->StartConsumer(stack_faces,
                               mutex,
                               processed_face_count,
                               vector_faces_output);

    consumer_6_ = std::make_shared<Consumer>(6);
    consumer_6_->StartConsumer(stack_faces,
                               mutex,
                               processed_face_count,
                               vector_faces_output);


	sub = it.subscribe("image", 1,
                           &ModelInterfaceNode::ConsumerCallback,
                           this);

    pub_image_ = it.advertise("/image_processed", 12);
}


void
ModelInterfaceNode::ConsumerCallback(const sensor_msgs::ImageConstPtr& msg)
{
    std::cout << "Image message is received." << std::endl;

    // Conversion ROS image message to cv::Mat type:
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}

    auto start_1 = high_resolution_clock::now();

	cv::Mat input_image = cv_ptr->image;
	std::cout << "Input image width:" << input_image.rows
	<< " height:" << input_image.cols <<  std::endl;

	FaceDetector::detectAndDraw(input_image,
                                cascade_face_detection_model,
                                scale, vector_face_and_id);

    auto stop_1 = high_resolution_clock::now();
    auto duration_1 = duration_cast<milliseconds>(stop_1 - start_1);
    std::cout << "Face detection time: " << duration_1.count() << " ms" << std::endl;

    size_t count_detected_face = vector_face_and_id.size();
    std::cout << "Detected face count: " << count_detected_face << std::endl;

    // process images parallel with Thrust library:
    auto PreProcess = [](std::pair<size_t, cv::Mat>& face_and_id)
            -> std::pair<size_t, cv::Mat> {
        face_and_id.second = PreProcessImage(face_and_id.second);
        return face_and_id;
    };
    thrust::transform(thrust::system::tbb::par,
                      vector_face_and_id.begin(),
                      vector_face_and_id.end(),
                      vector_face_and_id.begin(),
                      PreProcess);

    auto start_2 = high_resolution_clock::now();
    // Put all faces into the stack in order to be taken by consumer threads:
    mutex.lock();
    for(const auto& face_and_id:vector_face_and_id)
    {
        Face face;
        face.face_id = face_and_id.first;
        face.face_img = face_and_id.second;
        face.processed = false;

        stack_faces.push_back(face);
    }
    //std::cout << stack_faces.size() << std::endl;
    mutex.unlock();
    vector_face_and_id.clear();

    // Wait until all faces are classified by consumer threads
    while(count_detected_face != processed_face_count) {
    }

    // All consumer threads push result into the 'vector_faces_output'
    for (const auto& face:vector_faces_output)
        std::cout << "Face id: " << face.face_id << " | Class name: " << face.class_name
        << " | Probability: " << face.prob
        <<  " | Consumer id: "
        << face.consumer_id << std::endl;

    auto stop_2 = high_resolution_clock::now();
    auto duration_2 = duration_cast<milliseconds>(stop_2 - start_2);
    std::cout << "Classification time: " << duration_2.count() << " ms" << std::endl;

    // Publish the result image:
    std_msgs::Header  header;
    header.frame_id = "camera_frame";
    header.stamp = ros::Time::now();
    cv_bridge::CvImage cvImage(header, "bgr8", input_image);
    pub_image_.publish(cvImage.toImageMsg());
    std::cout << "Processed image is published." << std::endl;

    // Clear frame state:
    processed_face_count = 0;
    vector_faces_output.clear();

	std::cout << "################################################################################" << std::endl;
}



cv::Mat
ModelInterfaceNode::PreProcessImage(const cv::Mat& input_image)
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
