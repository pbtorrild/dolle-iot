#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/opencv.hpp>
#include <limcheck.h>
class ImagePublisher : public rclcpp::Node
{
  private:
    dolle_iot::limcheck limcheck_;
    image_transport::Publisher pub_;
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
    cv::VideoCapture cap;

  public:
    bool load_camera(){
        
        cap.set(cv::CAP_PROP_FRAME_WIDTH,640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,420);
        cap.open(0,cv::CAP_V4L);
        //cap.set(cv::CAP_PROP_FPS,60);
        if(cap.isOpened()){
            return true;
        }
        return false;
    }

    void start_publisher(){
        while (true)
        {
            cv::Mat frame;
            cap >> frame;
            if(frame.empty()){
            break;
            } 
            cv::Mat vision= limcheck_.apply(frame.clone());
            pub_.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", vision).toImageMsg());
        }
    }

    ImagePublisher() : Node("stigemaskine2_cam0")
    {      
        pub_ = image_transport::create_publisher(this, "stigemaskine2/limcheck/cam0", custom_qos);
        
        if(load_camera()){
            start_publisher();
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