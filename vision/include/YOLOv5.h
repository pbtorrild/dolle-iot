#ifndef YOLO_V5_H
#define YOLO_V5_H

#include <opencv2/opencv.hpp>

namespace vision{
    class YOLOv5
    {
    private:
        /* data */
    public:
        YOLOv5(/* args */);
        ~YOLOv5();

        cv::Mat format_input(cv::Mat source){
            // put the image in a square big enough
            int col = source.cols;
            int row = source.rows;
            int _max = MAX(col, row);
            cv::Mat resized = cv::Mat::zeros(_max, _max, CV_8UC3);
            source.copyTo(resized(cv::Rect(0, 0, col, row)));
            
            // resize to 640x640, normalize to [0,1[ and swap Red and Blue channels
            cv::Mat result;
            cv::dnn::blobFromImage(source, result, 1./255., cv::Size(source.cols, source.rows), cv::Scalar(), true, false);
        
            return result;
        }
    };

    YOLOv5::YOLOv5(/* args */)
    {
    }

    YOLOv5::~YOLOv5()
    {
    }
}
    



#endif