#ifndef RECORDING_H
#define RECORDING_H
#include <chrono>
#include <functional>
#include <memory>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <string.h>
#include <rclcpp/rclcpp.hpp>

namespace dolle_iot{
    namespace vision{
        class record : public rclcpp::Node{
            private:
                
                size_t camera_id_=0;
                cv::VideoCapture video_stream;
                rclcpp::TimerBase::SharedPtr timer_;
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
                size_t count_;

                uint32_t seq_=0;

                //Return encoding format from cv::Mat::type()
                std::string get_encoding(int type){
                    std::string rtn;
                    u_char depth = type & CV_MAT_DEPTH_MASK;
                    u_char chans = 1 + (type >> CV_CN_SHIFT);
                    switch ( depth ) {
                        case CV_8U:  rtn = "8U"; break;
                        case CV_8S:  rtn = "8S"; break;
                        case CV_16U: rtn = "16U"; break;
                        case CV_16S: rtn = "16S"; break;
                        case CV_32S: rtn = "32S"; break;
                        case CV_32F: rtn = "32F"; break;
                        case CV_64F: rtn = "64F"; break;
                        default:     rtn = "User"; break;
                    }

                    rtn+='C';
                    rtn+= (chans+'0');

                    return rtn;
                }
                //Retuns the data as a vector
                std::vector<unsigned char>get_data(unsigned char* arr){
                    std::vector<unsigned char> rtn;
                    size_t arr_size = *(&arr + 1) - arr;
                    for (size_t i = 0; i < arr_size; i++)
                    {
                        rtn.push_back(arr[i]);
                    }
                    
                    return rtn;
                }
            public:

                record(uint8_t camera_id=0) : Node("camera_"+camera_id), count_(0)
                {
                    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("stigemaskine1/camera_"+std::to_string(camera_id)+"/image_raw", 10);
                    camera_id_ =camera_id;
                    //Open specified camera
                    video_stream.open(camera_id_);
                    if (!video_stream.isOpened()) {
                        std::cerr << "ERROR! Unable to open camera" << std::endl;
                    }
                    //Calculates the number of milis betweem frames and call the image callback if we cant get the fps from camera we asume 30 :D 
                    auto tbf =(1000/(video_stream.get(cv::CAP_PROP_FPS)==0) ? 30 : video_stream.get(cv::CAP_PROP_FPS)) * std::chrono::milliseconds();
                    //Call to publish frame in sync with fps:
                    timer_ = this->create_wall_timer(tbf, std::bind(&record::image_callback, this));
                }
                ~record(){
                    video_stream.release();
                }
                void image_callback(){
                    cv::Mat frame;
                    video_stream >> frame;
                    publish_img(frame);
                }
                void publish_img(cv::Mat src){

                    auto msg_=sensor_msgs::msg::Image();
                    //std_msgs/Header header
                    msg_.header.stamp=this->rclcpp::Node::now();
                    msg_.header.frame_id=camera_id_;
                    //uint32 height
                    msg_.height=src.rows;
                    //uint32 width
                    msg_.width= src.cols;
                    //std::string encoding
                    msg_.encoding = get_encoding(src.type());
                    //uint8 is_bigendian
                    if(__BYTE_ORDER==__BIG_ENDIAN){
                        msg_.is_bigendian=true;
                    }
                    else{msg_.is_bigendian=false;}
                    //uint32 step 
                    msg_.step=src.cols*src.channels();
                    //uint8[] data
                    msg_.data=get_data(src.data);

                    publisher_->publish(msg_);
                }
        };
    }
}


#endif // RECORDING_H
