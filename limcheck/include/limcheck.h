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
        bool enable_preview=true; 
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
                if(plank_locations.begin()->id%20==0){                    
                    cv::imwrite("/home/petert/dev_ws/src/dolle-iot/limcheck/data/raw/"+std::to_string(plank_locations.begin()->id)+"_"+std::to_string(id_location_count)+".jpg",cv::Mat(src,plank_locations.begin()->pos));
                    cv::waitKey(5);
                    id_location_count++;
                }               
                
            }
            else{id_location_count =0;}
            //mark newest object
            for(vision::object plank : plank_locations){
                if(enable_preview){
                    cv::imshow("Vision alg",src(plank.pos));
                    cv::waitKey(1);
                }
                cv::rectangle(src,plank.pos,{255,0,0},2);
            }
            
            return src;
        }

        //Abstraction of data from image to rectangles
        void find_planks(cv::Mat src){
            /*################## FILTER ##################*/
                /*  PLANK CARACTERISTICS 
                    * Planks are are larger than 200x100 px
                    * Have origen below 340 y (upper part of img)
                    * Skip the first position, so more right than 20x
                    * New objects are always comming from the left side
                    * There can max be 2 objects on an image
                */ 
            
            object_detector_->apply(src,src);
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::threshold(src,src,254,255,0); //Remove shaddows
            cv::findContours(src,contours, hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
            plank_locations.clear();

            for (std::vector<cv::Point> contour : contours)
            {   
                cv::Rect rect_ = cv::boundingRect(contour);
                
                if(rect_.width<150){continue;}
                if(rect_.height<50){continue;}
                if(rect_.y>340){continue;}
                if(rect_.x<20){continue;}
                //Check if the object is new:
                if(rect_.x<latest_x){
                    id_count++;
                }
                latest_x=rect_.x;
                plank_locations.push_back(vision::object(id_count,rect_));
                vision::object(id_count,rect_).get_info();
            }

        }

        void find_dowels(cv::Mat src){
            /*################## FILTER ##################*/
                /*  PLANK CARACTERISTICS 
                    * dowels are circular

                */


        }
        
        

        limcheck(){
            std::cout << "id;c.x;c.y;width;height" << std::endl;
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