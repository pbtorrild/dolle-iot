#ifndef LIMCHECK_H
#define LIMCHECK_H

#include <opencv2/opencv.hpp>
#include <boost/atomic.hpp>
#include <terminal-msgs.h>
#include <yolo_v5.h>
#include <stdlib.h>
#include <time.h>
#include <tuple> 
#include <dolle_vision.h>
#include <math.h> 

namespace dolle_iot
{
    class limcheck
    {
    private:
        yolo yolo_v5;
        bool enable_preview=false; 
        vision::centre img_centre; 
        //object detection
        cv::Ptr<cv::BackgroundSubtractorKNN> object_detector_;
        int num_found_obj=0;
        //object tracking
        uint32_t id_count=1;
        int latest_x; // Maximum number of possible objects in image;
        std::vector<std::vector<std::vector<vision::object>>> glue_pos;
        int id_location_count=0;

    public:
        //location of each ROI
        std::vector<vision::object> plank_locations;
        //Vector of unique planks that contains a vector for each dowel, 
        //which in turn contains a vector of glue positions
        std::vector<std::vector<std::vector<vision::object>>> apply(cv::Mat src){
            if(img_centre.x<0){
                img_centre=vision::get_centre(src);
            }
            find_planks(src);
            glue_pos.clear();
            if(!plank_locations.empty()){
                for(vision::object plank : plank_locations){
                    glue_pos.push_back(find_glue(src(plank.pos)));
                }         
            }
            else{id_location_count =0;}
            return glue_pos;
        }

        //Abstraction of data from image to rectangles
        void find_planks(cv::Mat src){
            /*################## FILTER ##################*/
                /*  PLANK CHARACTERISTICS 
                    * Planks are are larger than 200x100 px
                    * Have origen below 340 y (upper part of img)
                    * Skip the first position,skip images that are on the left, so more right or left than 5px from the image boarder
                    * New objects are always coming from the left side
                    * There can max be 2 objects on an image
                */ 
            
            object_detector_->apply(src,src);
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::threshold(src,src,254,255,0); //Remove shadows
            cv::findContours(src,contours, hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
            plank_locations.clear();

            for (std::vector<cv::Point> contour : contours)
            {   
                cv::Rect rect_ = cv::boundingRect(contour);
                
                if(rect_.width<200){continue;}
                if(rect_.height<100){continue;}
                if(rect_.y>340){continue;}
                if(rect_.x<5 || (rect_.x+rect_.width)>(src.cols-5)){continue;}
                //Check if the object is new:
                if(rect_.x<latest_x){
                    id_count++;
                }
                latest_x=rect_.x;
                plank_locations.push_back(vision::object(id_count,rect_));
            }

        }

        std::vector<std::vector<vision::object>> find_glue(cv::Mat src){
            yolo_v5.pre_process(src);
            std::vector<vision::object> dowels;
            std::vector<vision::object> glues;
            //Sort detected objects into dowels and glue
            for(vision::object detected_obj : yolo_v5.post_process(src)){
                if(yolo_v5.get_classes().at(detected_obj.id)=="lim"){
                    glues.push_back(detected_obj);
                }
                else{
                    dowels.push_back(detected_obj);
                }
            }
            std::vector<std::vector<vision::object>> glue_pos_(dowels.size());
            for(vision::object glue : glues){
                //Find closes dowel
                vision::centre glue_c = vision::get_centre(glue.pos);
                //Init euclidean distance as maximum possible
                int nearest_dowel, current_dowel=0;
                float dist = sqrt(pow(src.cols,2)+pow(src.rows,2));
                for(vision::object dowel : dowels){
                    vision::centre dowel_c = vision::get_centre(dowel.pos);
                    float dist_current =sqrt(pow(glue_c.x-dowel_c.x,2)+pow(glue_c.y-dowel_c.y,2));
                    if(dist_current<=dist){
                        nearest_dowel=current_dowel;
                    }
                    current_dowel++;
                }
                glue_pos_.at(nearest_dowel).push_back(glue);
            }
            return glue_pos_;
        }

        cv::Mat illustrate_vision(cv::Mat src){
            int ptr_=0;
            for(vision::object plank : plank_locations){
                //Draw plank pos onto image
                cv::rectangle(src, plank.pos, dolle_iot::DARK_RED, dolle_iot::THICKNESS);
                for(std::vector<vision::object>dowel :glue_pos.at(ptr_)){
                    //Draw glue onto image
                    for(vision::object glue : dowel){
                        int left = glue.pos.x+plank.pos.x;
                        int top = glue.pos.y+plank.pos.y;
                        int width = glue.pos.width;
                        int height = glue.pos.height;
                        // Draw bounding box.
                        cv::rectangle(src, cv::Point(left, top), cv::Point(left + width, top + height), dolle_iot::RED, dolle_iot::THICKNESS);
                    }
                }
                ptr_++;
            }
            return src;
        }

        limcheck(){
            object_detector_=cv::createBackgroundSubtractorKNN();
            if(enable_preview){
                cv::namedWindow("Vision alg");
                cv::waitKey(1);
            }

        };
        ~limcheck(){
            cv::destroyAllWindows();
        };
    };

}

#endif