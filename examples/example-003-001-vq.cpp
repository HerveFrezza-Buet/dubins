#include <opencv2/opencv.hpp>
#include <sstream>
#include <demo2d.hpp>
#include <dubins.hpp>


#define POSE_RADIUS      11
#define POSE_LENGTH     .4

#define THICKNESS_POSE   3
#define COLOR_POSE1  cv::Scalar(  0,   0,   0)
#define COLOR_POSE2  cv::Scalar(  0,   0, 150)
#define COLOR_PATH   cv::Scalar(170, 170, 170)
#define COLOR_CIRCLE cv::Scalar(220, 220, 220)

#define ALPHA_COEF           .1
#define ALPHA_SLIDER_SIZE   100 

struct Param {
  double R()              const {return .5;}
  double tol_angle()      const {return dubins::to_rad(1);}
  double tol_distance_2() const {return .01*.01;}
};
  
enum class Mode : char {
  Free = 'f',
  Position  = 'p',
  Angle = 'a'
};

class OnMouseInfo {
private:
  Mode mode = Mode::Free;

public:
  double x      = 0;
  double y      = 0;
  double theta  = 0;
  double alpha  = 0;
  const demo2d::opencv::Frame& frame;
  
  OnMouseInfo(const demo2d::opencv::Frame& frame) : frame(frame) {};

  void set_alpha(double a) {
    alpha = a;
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
      break;
    case Mode::Angle:
      this->theta = .5 * dubins_PI * (P.x + P.y);
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
  info.set_alpha(ALPHA_COEF*value/(double)ALPHA_SLIDER_SIZE);
}

int main(int argc, char* argv[]) {

  int slider = 0;
  
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
  auto image = cv::Mat(800, 1000, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .1*image.size().width, true);

  OnMouseInfo info(frame);
  cv::setMouseCallback("image", on_mouse, reinterpret_cast<void*>(&info));
  std::ostringstream ostr;
  ostr << "alpha [0, " << ALPHA_COEF << ']';
  cv::createTrackbar(ostr.str(), "image", &slider, 100, on_trackbar, reinterpret_cast<void*>(&info));

  dubins::pose<Param> w;
  std::cout << std::endl
	    << "Press ESC to quit." << std::endl
	    << "right click -> toggle position mode." << std::endl
	    << "middle click -> toggle angle mode." << std::endl
	    << std::endl;
  int keycode = 0;
  while(keycode != 27) {
    dubins::pose<Param> xi {info.x, info.y, info.theta};
      
    image = cv::Scalar(255, 255, 255);

    // Circles
    auto [c1, c2] =  w.left_right_circles(Param().R());
    auto [c3, c4] = xi.left_right_circles(Param().R());
    
    dubins::draw(image, frame, c1, COLOR_CIRCLE, 1);
    dubins::draw(image, frame, c2, COLOR_CIRCLE, 1);
    dubins::draw(image, frame, c3, COLOR_CIRCLE, 1);
    dubins::draw(image, frame, c4, COLOR_CIRCLE, 1);
      
    // Dubins paths
    auto diff = (xi - w); // This is for display
    dubins::draw(image, frame, diff.first.first,  cv::Scalar(170, 255, 170), 3); // The best Dubins path
    dubins::draw(image, frame, diff.second.first, cv::Scalar(170, 170, 255), 3); // The reserse Dubins path.

    // Online vq updating rule.
    w += info.alpha*(xi - w);
      
    // We draw the target pose.
    dubins::draw(image, frame, xi, POSE_RADIUS, POSE_LENGTH, COLOR_POSE1, THICKNESS_POSE);
    dubins::draw(image, frame,  w, POSE_RADIUS, POSE_LENGTH, COLOR_POSE2, THICKNESS_POSE);
      
      
    // Let us display the result.
    cv::imshow ("image",image);
    keycode = cv::waitKey(10) & 0xFF;
  }

  
  return 0;
}

  
