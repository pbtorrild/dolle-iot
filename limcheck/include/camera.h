#if !defined(CAMERA_H)
#define CAMERA_H

#include <iostream>
#include <dolle_vision.h>
#include <opencv2/opencv.hpp>

namespace dolle_iot
{
    class Camera
    {
    private:
        /* data */
        cv::VideoCapture cap;
    protected:
        int width, height, fps;

    public:
        // Stores centre coordinates of the image frame
        vision::centre centre;
        
        // Loads the camera device with specified settings
        bool load(const char * path, int frame_width, int frame_height, int frame_rate);
        // Returns now frame from camera device
        cv::Mat read();
        
        ~Camera();
    };
    
    bool Camera::load(const char * path, int frame_width = 640, int frame_height = 480, int frame_rate = 30)
    {
        // Open camera:
        cap.open(path,cv::CAP_V4L);
        if(!cap.isOpened()){return false;}

        // Set desired camera settings:
        cap.set(cv::CAP_PROP_FRAME_WIDTH,frame_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,frame_height);
        cap.set(cv::CAP_PROP_FPS,frame_rate);

        // Get actual camera settings:
        width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        fps = cap.get(cv::CAP_PROP_FPS);

        // Notify if the settings wasn't set to desired values
        if (width != frame_width || height != frame_height || fps != frame_rate)
        {
            std::cout << "Camera settings could not be set to desired values. \n";
            std::cout << "Camera is running video of " << width << " by " << height << " px at " << fps << " fps \n";
        }

        // Set centre
        centre = {width/2,height/2};
        return true;
    }

    cv::Mat Camera::read()
    {
        cv::Mat frame;
        cap >> frame;
        return frame;
    }
    
    Camera::~Camera()
    {
        cap.release();
    }
    
}


#endif // CAMERA.H
