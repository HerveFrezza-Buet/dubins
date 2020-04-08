#include <opencv2/opencv.hpp>
#include <demo2d.hpp>
#include <dubins.hpp>

#define POSE_RADIUS      11
#define POSE_LENGTH     .4

#define THICKNESS_POSE   3
#define THICKNESS_PATH   3

#define COLOR_POSE        cv::Scalar(  0,   0,   0)
#define COLOR_INSENSITIVE cv::Scalar(170, 170, 170)
#define COLOR_PATH        cv::Scalar(  0,   0, 170)

#define RHO .5

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
  double x = 0;
  double y = 0;
  double theta = 0;
  const demo2d::opencv::Frame& frame;
  
  OnMouseInfo(const demo2d::opencv::Frame& frame) : frame(frame) {};

  operator bool() const {bool res = changed; changed = false; return res;}

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

int main(int argc, char* argv[]) {

  
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
  auto image = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .1*image.size().width, true);

  OnMouseInfo info(frame);
  cv::setMouseCallback("image", on_mouse, reinterpret_cast<void*>(&info));

  dubins::Pose p1 {};
  auto [r1, l1] = p1.left_right_circles(RHO);
  
  

  std::cout << std::endl
	    << "Press ESC to quit." << std::endl
	    << "right click -> toggle position mode." << std::endl
	    << "middle click -> toggle angle mode." << std::endl
	    << std::endl;
  int keycode = 0;
  while(keycode != 27) {
    if(info) {
      dubins::Pose p2 {info.x, info.y, info.theta};
      auto [r2, l2] = p2.left_right_circles(RHO);
      
      image = cv::Scalar(255, 255, 255);
      
      // We draw the circles
      dubins::draw(image, frame, r1, COLOR_INSENSITIVE, 1);
      dubins::draw(image, frame, l1, COLOR_INSENSITIVE, 1);
      dubins::draw(image, frame, r2, COLOR_INSENSITIVE, 1);
      dubins::draw(image, frame, l2, COLOR_INSENSITIVE, 1);
      
      // We draw the path
      dubins::draw(image, frame, dubins::path(p1, p2, RHO), COLOR_PATH, THICKNESS_PATH);
      
      // We draw the start pose.
      dubins::draw(image, frame, p1, POSE_RADIUS, POSE_LENGTH, COLOR_POSE, THICKNESS_POSE);
      
      // We draw the destination pose.
      dubins::draw(image, frame, p2, POSE_RADIUS, POSE_LENGTH, COLOR_POSE, THICKNESS_POSE);
      
      // Let us display the result.
      cv::imshow ("image",image);
    }
    keycode = cv::waitKey(10) & 0xFF;
  }

  
  return 0;
}

  
