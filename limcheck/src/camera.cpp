#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>
#include <time.h>
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
  cap.set(cv::CAP_PROP_FRAME_WIDTH,640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT,420);
  cap.set(cv::CAP_PROP_FPS,60);
  cv::waitKey(1000);
  if(!cap.isOpened()){
    terminal_msgs::error("Could not open camera");
    return -1;
  } 
  
  dolle_iot::limcheck limcheck;
  while (running)
  {
    auto t0 = std::chrono::high_resolution_clock::now();
    cv::Mat frame;
    cap >> frame;
    if(frame.empty()){
      break;
    } 
    cv::Mat vision= limcheck.apply(frame.clone());
    if(preview){
      auto t1 = std::chrono::high_resolution_clock::now();
      int time_dif = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
      std::string fps("FPS: "+std::to_string((1000/time_dif)));
      cv::putText(vision,fps,cv::Point(20,20),cv::FONT_HERSHEY_DUPLEX,0.6,cv::Scalar(0,255,0),2,false);
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