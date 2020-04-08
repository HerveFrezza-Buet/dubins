#include <opencv2/opencv.hpp>
#include <demo2d.hpp>
#include <dubins.hpp>


#define POSE_RADIUS      11
#define POSE_LENGTH     .4

#define THICKNESS_POSE   3
#define COLOR_POSE1  cv::Scalar(  0,   0,   0)
#define COLOR_POSE2  cv::Scalar(  0,   0, 150)



enum class Mode : char {
  Free = 'f',
  Position  = 'p',
  Angle = 'a'
};

class OnMouseInfo {
private:
  Mode mode = Mode::Free;
  mutable bool changed = true;

public:
  double x      = 0;
  double y      = 0;
  double theta  = 0;
  double alpha  = 0;
  const demo2d::opencv::Frame& frame;
  
  OnMouseInfo(const demo2d::opencv::Frame& frame) : frame(frame) {};

  operator bool() const {bool res = changed; changed = false; return res;}

  void set_alpha(double a) {
    alpha = a;
    changed = true;
  }
  
  void mode_button(Mode m) {
    if(m == mode)
      mode = Mode::Free;
    else
      mode = m;
  }
  
  void mouse_at(int x, int y) {
    auto P = frame(cv::Point(x, y));
    switch(mode) {
    case Mode::Free:
      break;
    case Mode::Position:
      this->x = P.x;
      this->y = P.y;
      this->changed = true;
      break;
    case Mode::Angle:
      this->theta = .5 * dubins_PI * (P.x + P.y);
      this->changed = true;
      break;
    }
    
  }
};

void on_mouse(int event, int x, int y, int, void* user_data) {
  auto& info = *(reinterpret_cast<OnMouseInfo*>(user_data));
  if(event == cv::EVENT_LBUTTONDOWN)
    info.mode_button(Mode::Position);
  else if(event == cv::EVENT_RBUTTONDOWN)
    info.mode_button(Mode::Free);
  else if(event == cv::EVENT_MBUTTONDOWN)
    info.mode_button(Mode::Angle);
  info.mouse_at(x, y);
}

void on_trackbar(int value, void* user_data) {
  auto& info = *(reinterpret_cast<OnMouseInfo*>(user_data));
  info.set_alpha(value*1e-4);
}

int main(int argc, char* argv[]) {

  int slider = 500;
  
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
  auto image = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .1*image.size().width, true);

  OnMouseInfo info(frame);
  cv::setMouseCallback("image", on_mouse, reinterpret_cast<void*>(&info));
  cv::createTrackbar("1e4*alpha", "image", &slider, 1000, on_trackbar, reinterpret_cast<void*>(&info));
  
  std::cout << std::endl
	    << "Press ESC to quit." << std::endl
	    << "right click -> toggle position mode." << std::endl
	    << "middle click -> toggle angle mode." << std::endl
	    << std::endl;
  int keycode = 0;
  while(keycode != 27) {
    if(info) {
      dubins::Pose p2 {info.x, info.y, info.theta};
      
      image = cv::Scalar(255, 255, 255);
      
      
      // We draw the start pose.
      dubins::draw(image, frame, p2, POSE_RADIUS, POSE_LENGTH, COLOR_POSE1, THICKNESS_POSE);
      
      
      // Let us display the result.
      cv::imshow ("image",image);
    }
    keycode = cv::waitKey(10) & 0xFF;
  }

  
  return 0;
}

  
