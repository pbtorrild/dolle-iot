#if !defined(DOLLE_VISION_H)
#define DOLLE_VISION_H
#include <opencv2/opencv.hpp>
namespace dolle_iot{
    // Text parameters.
	const float FONT_SCALE = 0.3;
	const int FONT_FACE = cv::FONT_HERSHEY_SIMPLEX;
	const int THICKNESS = 1;
    
    // Colors. (Note in BGR)
    cv::Scalar RED = cv::Scalar(45, 16, 215);
    cv::Scalar DARK_GRAY = cv::Scalar(81, 88, 92);
    cv::Scalar LIGHT_GRAY = cv::Scalar(228, 232, 238);
    cv::Scalar DARK_RED = cv::Scalar(23, 16, 148);
    cv::Scalar BLACK = cv::Scalar(0, 0, 0);
    cv::Scalar WHITE = cv::Scalar(255, 255, 255);

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

}


#endif // DOLLE_VISION
