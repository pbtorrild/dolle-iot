#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "limcheck/msg/object_pos.hpp"

#include <opencv2/opencv.hpp>
#include <limcheck.h>
class ImagePublisher : public rclcpp::Node
{
  private:
    dolle_iot::limcheck limcheck_;
    image_transport::Publisher image_pub;
    rclcpp::Publisher<limcheck::msg::ObjectPos>::SharedPtr data_pub;
    rmw_qos_profile_t custom_qos = {
      RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      10,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    //Open Camera
    int cam_num;
    cv::VideoCapture cap;

  public:
    bool load_camera(int cam_num_ = 0){
        
        cap.set(cv::CAP_PROP_FRAME_WIDTH,640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,420);
        cap.open(cam_num_,cv::CAP_V4L);
        //cap.set(cv::CAP_PROP_FPS,60);
        if(cap.isOpened()){
            return true;
        }
        return false;
    }

    void start_publishers(){
        while (true)
        {
            cv::Mat frame;
            cap >> frame;
            if(frame.empty()){
              break;
            } 
            std::vector<std::vector<std::vector<dolle_iot::vision::object>>> glue_pos = limcheck_.apply(frame.clone());
            for(std::vector<std::vector<dolle_iot::vision::object>> plank : glue_pos){  
                limcheck::msg::ObjectPos msg;
                int ptr_=0;
                int dowel_ptr=0;
                for(std::vector<dolle_iot::vision::object>dowel : plank){
                    msg.roi.x = limcheck_.plank_locations.at(dowel_ptr).pos.x;
                    msg.roi.y = limcheck_.plank_locations.at(dowel_ptr).pos.y;
                    msg.roi.w = limcheck_.plank_locations.at(dowel_ptr).pos.width;
                    msg.roi.h = limcheck_.plank_locations.at(dowel_ptr).pos.height;
                    dowel_ptr++;
                    msg.camera_num = 0;
                    msg.dowel =ptr_;
                    std::vector<limcheck::msg::ImagePos> glue_;
                    for(dolle_iot::vision::object glue : dowel){
                      limcheck::msg::ImagePos pos_;
                      pos_.x = glue.pos.x;
                      pos_.y = glue.pos.y;
                      pos_.w = glue.pos.width;
                      pos_.h = glue.pos.height;
                      glue_.push_back(pos_);
                    }
                    msg.glue=glue_;
                    data_pub->publish(msg);
                }
                ptr_++;
                
            }

            cv::Mat vision = limcheck_.illustrate_vision(frame.clone());
            image_pub.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", vision).toImageMsg());
        }
    }

    ImagePublisher() : Node("limcheck")
    {   
        this->declare_parameter<int>("camera",0);
        this->get_parameter("camera",cam_num);
        data_pub = this->create_publisher<limcheck::msg::ObjectPos>("limcheck/data", 10);
        image_pub = image_transport::create_publisher(this, "limcheck/cam" + std::to_string(cam_num) , custom_qos);
        
        if(load_camera(cam_num)){
            start_publishers();
        }
    }
    ~ImagePublisher(){
      cap.release();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ImagePublisher> imagePublisher = std::make_shared<ImagePublisher>();
    rclcpp::spin(imagePublisher);
    rclcpp::shutdown();
    return 0;
}