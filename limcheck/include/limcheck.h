#ifndef LIMCHECK_H
#define LIMCHECK_H

#include <opencv2/opencv.hpp>
#include <boost/atomic.hpp>
#include <terminal-msgs.h>
#include <stdlib.h>
#include <time.h>
#include <tuple> 
#include <recording.h>
namespace dolle_iot
{
    namespace vision{
        
        struct centre{
                int x=-1;
                int y=-1;
            };
        centre get_centre(cv::Mat img){
            return centre {img.cols/2,img.rows/2};
        }
        centre get_centre(cv::Rect rect_){
            return centre {rect_.x+rect_.width/2,rect_.y+rect_.height/2};
        }
        struct object{
            int32_t id=-1;
            cv::Rect pos;
            object(int64_t id_,cv::Rect rect_){
                id=id_;
                pos=rect_;
            }
            object(cv::Rect rect_){
                pos=rect_;
            }
            static bool compare(object obj1, object obj2){
                return (obj1.pos.x<obj2.pos.x);
            }
            //Provides data as terminal output with default seperator ';'
            void get_info(char s = ';'){
                centre c=get_centre(pos);
                std::cout << id <<s<< c.x <<s<< c.y <<s<< pos.width <<s<< pos.height << std::endl;
            }
        };
        

    }
    class limcheck
    {
    private:
        bool enable_preview=false; 
        vision::centre img_centre; 
        //object detection
        cv::Ptr<cv::BackgroundSubtractorKNN> object_detector_;
        int num_found_obj=0;
        //object tracking
        uint32_t id_count=1;
        int latest_x; // Maximum number of possible objects in image;
        std::vector<vision::object> plank_locations;
        int id_location_count=0;

    public:
        cv::Mat apply(cv::Mat src){
            if(img_centre.x<0){
                img_centre=vision::get_centre(src);
            }
            find_planks(src);
            
            if(!plank_locations.empty()){
                if(plank_locations.begin()->id%25==0){                    
                    cv::imwrite("/home/petert/dev_ws/src/dolle-iot/limcheck/data/raw/"+std::to_string(plank_locations.begin()->id)+"_"+std::to_string(id_location_count)+".jpg",cv::Mat(src,plank_locations.begin()->pos));
                    id_location_count++;
                }              
                
            }
            else{id_location_count =0;}
            //mark newest object
            for(vision::object plank : plank_locations){
                cv::Scalar color = cv::Scalar(255,0,0);
                cv::rectangle(src,plank.pos,color,2);
                cv::putText(src,"ID: "+std::to_string(plank.id),cv::Point(plank.pos.x,plank.pos.y),cv::FONT_HERSHEY_DUPLEX,1,color,2,false);
            }
            
            return src;
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
                vision::object(id_count,rect_).get_info();
            }

        }

        std::vector<std::vector<cv::Point>> find_glue(cv::Mat src){
            cv::cvtColor(src,src,cv::COLOR_BGR2GRAY);
            cv::threshold(src,src,230,255,0); //Remove non white elements
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::threshold(src,src,254,255,0);
            cv::findContours(src,contours, hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
            if(enable_preview){
                cv::imshow("Vision alg",src);
                cv::waitKey(1);
            }
            return contours;
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