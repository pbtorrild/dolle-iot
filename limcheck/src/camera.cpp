#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include <limcheck.h>
#include <terminal-msgs.h>

bool running =true;
void sigint_callback(int){
  running =false;
}


int main()
{
  bool preview =true;
  signal(SIGINT,sigint_callback);
  int cam_id=0;

  cv::VideoCapture cap(cam_id);
  cv::waitKey(200);
  if(!cap.isOpened()){
    terminal_msgs::error("Coud not open camera");
    return -1;
  } 
  dolle_iot::limcheck limcheck;
  while (running)
  {
    cv::Mat frame;
    cap >> frame;

    if(frame.empty()){
      break;
    } 
    cv::Mat vision= limcheck.apply(frame.clone());
    if(preview){
      cv::imshow("Preview",vision);
      char c = (char)cv::waitKey(1);
      if(c=='v'){
        cv::destroyAllWindows();
        preview=false;
      } 
    }  
  }
  cap.release();

  return 0;

}