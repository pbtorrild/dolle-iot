#include <limcheck.h>
#include <stdlib.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class glue_detector : public rclcpp::Node {
private:   
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    dolle_iot::limcheck vision_alg;
    const std::string obj_detect_param_threshold = "Object Detection Threshold";

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            terminal_msgs::error("cv_bridge exception: " + std::string(e.what()));
        }
        if(!cv_ptr->image.empty()){
            cv_ptr->image=vision_alg.apply(cv_ptr->image);
        }
        pub_.publish(cv_ptr->toImageMsg());
    }
   

public:
    glue_detector() : Node("image_converter") {
        //Set QoS
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
        pub_ = image_transport::create_publisher(this, "stigemaskine1/vision", custom_qos);
        sub_ = image_transport::create_subscription(this, "image_raw",
                std::bind(&glue_detector::imageCallback, this,std::placeholders::_1), "raw", custom_qos);
    }

    ~glue_detector()
    {
        cv::destroyAllWindows();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<glue_detector>());
    rclcpp::shutdown();
    return 0;
}