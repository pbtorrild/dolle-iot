#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "limcheck/msg/object_pos.hpp"

#include <opencv2/opencv.hpp>
#include <limcheck.h>
#include <camera.h>
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
    
    //Cameras:
    dolle_iot::Camera camera_1;
    int consecutive_read_errors=0;

    int32_t last_plank_id=-1;

  public:

    void start_publishers(){
            while (true)
            {
                cv::Mat frame = camera_1.read();
                if(frame.empty()){
                  consecutive_read_errors++;
                  if(consecutive_read_errors<=10){
                    continue;
                  }
                  break;
                }
                consecutive_read_errors=0;
                /*
                //Std: segmentation of image, using background-subtraction.
                limcheck_.find_planks(frame);                
                std::vector<dolle_iot::vision::object> planks = limcheck_.plank_locations;
                if (!planks.empty())
                {
                  for(dolle_iot::vision::object plank : planks){

                    if(plank.id <= last_plank_id){continue;}

                    dolle_iot::vision::centre plank_centre=dolle_iot::vision::get_centre(plank.pos);
                    //Is the plank centre within 10% of the image centre, then save that and select next plank to save
                    if(plank_centre.x > 0.9*camera_1.centre.x && plank_centre.x < 1.1*camera_1.centre.x){
                      //Save img
                      cv::imwrite("~/data/"+std::to_string(plank.id)+"_cam_1.jpg",frame.clone());
                      //Select next plank to save
                      last_plank_id = plank.id + 1 + (rand() % 50);
                                          
                    }
                  }     
                }
                */
                           
                cv::Mat vision = limcheck_.illustrate_vision(frame.clone());
                image_pub.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", vision).toImageMsg());
            }

        }
        
    ImagePublisher() : Node("data_check")
    {   
        this->declare_parameter("device_path", "/dev/video0");
        this->declare_parameter("image_topic", "limcheck/cam1");
        const char * device_path = this->get_parameter("device_path").get_parameter_value().get<std::string>().c_str();
        const char * image_topic = this->get_parameter("image_topic").get_parameter_value().get<std::string>().c_str();

        image_pub = image_transport::create_publisher(this, image_topic , custom_qos);

        if(camera_1.load(device_path)){
            start_publishers();
            RCLCPP_INFO(this->get_logger(), "Opened camera @ %s!", device_path);
        }
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