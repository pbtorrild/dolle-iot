#ifndef YOLO_V5_H
#define YOLO_V5_H
// Include Libraries.
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <terminal-msgs.h>
#include <dolle_vision.h>
class yolo
{
private:
	// Constants.
	const float INPUT_WIDTH = 640.0;
	const float INPUT_HEIGHT = 640.0;
	const float SCORE_THRESHOLD = 0.5;
	const float NMS_THRESHOLD = 0.45;
	const float CONFIDENCE_THRESHOLD = 0.45;

	// Text parameters.
	const float FONT_SCALE = 0.3;
	const int FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;
	const int THICKNESS = 1;

	// Colors. (Note in BGR)
    cv::Scalar RED = cv::Scalar(45, 16, 215);
    cv::Scalar DARK_GRAY = cv::Scalar(81, 88, 92);
    cv::Scalar LIGHT_GRAY = cv::Scalar(228, 232, 238);
    cv::Scalar DARK_RED = cv::Scalar(23, 16, 148);
    cv::Scalar BLACK = cv::Scalar(0, 0, 0);
    cv::Scalar WHITE = cv::Scalar(255, 255, 255);

    //Neural Network
    std::vector<std::string> class_list{"lim","dyvl"};
    cv::dnn::Net net;

    //Processing
    std::vector<cv::Mat> detections;

public:
	//Constructors
    yolo();
    yolo(std::string path_to_onnx, std::vector<std::string> labels);
	//Destructor
	~yolo();

	//Member functions
    //Cosmetic:
	void draw_label(cv::Mat& input_image, std::string label, int left, int top);
    cv::Mat display_efficiency_info(cv::Mat input_image);

    //Access:
    std::vector<std::string> get_classes(){
        return class_list;
    }
	void pre_process(cv::Mat input_image);
	cv::Mat post_process_cv_mat(cv::Mat input_image);
    std::vector<dolle_iot::vision::object> post_process(cv::Mat input_image);
};

yolo::yolo()
{
    net= cv::dnn::readNet("../ml_models/limcheck.onnx");
}
yolo::yolo(std::string path_to_onnx, std::vector<std::string> labels)
{
    net = cv::dnn::readNet(path_to_onnx);
    class_list.clear();
    for (std::string label : labels) {
        class_list.push_back(label);
    }
}

yolo::~yolo()
{
}

void yolo::draw_label(cv::Mat& input_image, std::string label, int left, int top)
{
	// Display the label at the top of the bounding box.
	int baseLine;
	cv::Size label_size = cv::getTextSize(label, FONT_FACE, FONT_SCALE, THICKNESS, &baseLine);
	// Put the label on image
	cv::putText(input_image, label, cv::Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, RED, THICKNESS);
}

cv::Mat yolo::display_efficiency_info(cv::Mat input_image)
{
    std::vector<double> layersTimes;
    double freq = cv::getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    std::string label = cv::format("Inference time : %.2f ms", t);
    cv::putText(input_image, label, cv::Point(20, 40), FONT_FACE, FONT_SCALE, RED);
    return input_image;
}

void yolo::pre_process(cv::Mat input_image)
{
    detections.clear();
	// Convert to blob.
	cv::Mat blob;
	cv::dnn::blobFromImage(input_image, blob, 1. / 255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);

	net.setInput(blob);

	// Forward propagate.
	net.forward(detections, net.getUnconnectedOutLayersNames());

}

cv::Mat yolo::post_process_cv_mat(cv::Mat input_image)
{
    // Initialize vectors to hold respective outputs while unwrapping detections.
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // Resizing factor.
    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;

    auto data = detections.at(0);    

    // Iterate through 25200 detections.
    for (int row = 0; row < data.size[1]; ++row)
    {
        float confidence = data.at<float>(0, row, 4);
        // Discard bad detections and continue.
        if (confidence >= CONFIDENCE_THRESHOLD)
        {
            //Find best class score.
            float max_class_score = 0.;
            int class_id = 0;
            for (int i = 5; i < data.size[2]; i++)
            {
                if (data.at<float>(0, row, i) >= max_class_score) {
                    max_class_score = data.at<float>(0, row, i);
                    class_id = i - 5;
                }
            }

            // Continue if the class score is above the threshold.
            if (max_class_score > SCORE_THRESHOLD)
            {
                // Store class ID and confidence in the pre-defined respective vectors.

                confidences.push_back(confidence);
                class_ids.push_back(class_id);

                // Center.
                float cx = data.at<float>(0, row, 0);
                float cy = data.at<float>(0, row, 1);
                // Box dimension.
                float w = data.at<float>(0, row, 2);
                float h = data.at<float>(0, row, 3);
                // Bounding box coordinates.
                int left = int((cx - 0.5 * w) * x_factor);
                int top = int((cy - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                // Store good detections in the boxes vector.
                boxes.push_back(cv::Rect(left, top, width, height));
            }

        }
    }

    // Perform Non Maximum Suppression and draw predictions.
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
    for (long unsigned int i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];

        int left = box.x;
        int top = box.y;
        int width = box.width;
        int height = box.height;
        // Draw bounding box.
        cv::rectangle(input_image, cv::Point(left, top), cv::Point(left + width, top + height), DARK_RED, THICKNESS);

        // Get the label for the class name and its confidence.
        std::string label = cv::format("%.2f", confidences[idx]);
        label = class_list[class_ids[idx]] + ":" + label;
        // Draw class labels.
        draw_label(input_image, label, left, top);
    }
    return input_image;
}

std::vector<dolle_iot::vision::object> yolo::post_process(cv::Mat input_image)
{
    // Initialize vectors to hold respective outputs while unwrapping detections.
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    std::vector<dolle_iot::vision::object> detected_objs;

    // Resizing factor.
    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;

    auto data = detections.at(0);    

    // Iterate through 25200 detections.
    for (int row = 0; row < data.size[1]; ++row)
    {
        float confidence = data.at<float>(0, row, 4);
        // Discard bad detections and continue.
        if (confidence >= CONFIDENCE_THRESHOLD)
        {
            //Find best class score.
            float max_class_score = 0.;
            int class_id = 0;
            for (int i = 5; i < data.size[2]; i++)
            {
                if (data.at<float>(0, row, i) >= max_class_score) {
                    max_class_score = data.at<float>(0, row, i);
                    class_id = i - 5;
                }
            }

            // Continue if the class score is above the threshold.
            if (max_class_score > SCORE_THRESHOLD)
            {
                // Store class ID and confidence in the pre-defined respective vectors.

                confidences.push_back(confidence);
                class_ids.push_back(class_id);

                // Center.
                float cx = data.at<float>(0, row, 0);
                float cy = data.at<float>(0, row, 1);
                // Box dimension.
                float w = data.at<float>(0, row, 2);
                float h = data.at<float>(0, row, 3);
                // Bounding box coordinates.
                int left = int((cx - 0.5 * w) * x_factor);
                int top = int((cy - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                // Store good detections in the boxes vector.
                boxes.push_back(cv::Rect(left, top, width, height));
            }

        }
    }

    // Perform Non Maximum Suppression and draw predictions.
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, indices);
    for (long unsigned int i = 0; i < indices.size(); i++)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        detected_objs.push_back(dolle_iot::vision::object(idx,box));
    }
    return detected_objs;
}

#endif