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
    int32_t last_plank_id=-1;
    dolle_iot::vision::centre img_centre;
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
                if(img_centre.x <0 || img_centre.y <0){
                  img_centre = dolle_iot::vision::get_centre(frame);
                }
                //Std: segmentation of image, using background-subtraction.

                limcheck_.find_planks(frame);
                std::vector<dolle_iot::vision::object> planks = limcheck_.plank_locations;
                std::vector<dolle_iot::vision::object> known_locations;
                
                for(dolle_iot::vision::object plank : planks){

                  if(plank.id <= last_plank_id){continue;}

                  dolle_iot::vision::centre plank_centre=dolle_iot::vision::get_centre(plank.pos);
                  //Is the plank centre within 10% of the image centre, then save that and select next plank to save
                  if(plank_centre.x > 0.9*img_centre.x && plank_centre.x < 1.1*plank_centre.x){
                    //Save img
                    cv::Mat save_image = frame(plank.pos);
                    if(!save_image.empty()){
                      cv::imwrite("~/data/"+std::to_string(plank.id)+"_cam_"+std::to_string(cam_num)+".jpg",save_image);
                      //Select next plank to save
                      last_plank_id = plank.id + 1 + (rand() % 50);
                    }
                                        
                  }
                }                
                cv::Mat vision = limcheck_.illustrate_vision(frame.clone());
                image_pub.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", vision).toImageMsg());
            }

        }
        
    ImagePublisher() : Node("data_check")
    {   
        this->declare_parameter<int>("camera",0);
        this->get_parameter("camera",cam_num);
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