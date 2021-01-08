#include <opencv2/opencv.hpp>
#include <demo2d.hpp>
#include <dubins.hpp>

#define RHO .5
#define ARG_COLOR cv::Scalar(  0,   0,   0)

class OnMouseInfo {
private:
  double x      = 0;
  double y      = 0;
  const demo2d::opencv::Frame& frame;

  mutable bool changed = true;

public:
  OnMouseInfo(const demo2d::opencv::Frame& frame) : frame(frame) {}
  

  operator bool() const {bool res = changed; changed = false; return res;}
  
  void mouse_at(int x, int y) {
    auto P = frame(cv::Point(x, y));
    this->x = P.x;
    this->y = P.y;
    changed = true;
  }
};

void on_mouse(int event, int x, int y, int, void* user_data) {
  auto& info = *(reinterpret_cast<OnMouseInfo*>(user_data));
  info.mouse_at(x, y);
}


int main(int argc, char* argv[]) {
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  auto image = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .1*image.size().width, true);

  OnMouseInfo info(frame);
  cv::setMouseCallback("image", on_mouse, reinterpret_cast<void*>(&info));

  demo2d::Point  O  {0., 0.}; // Origin
  dubins::Circle C {O, RHO};
  dubins::Circle M {C};

  
  std::cout << std::endl
	    << "Press ESC to quit." << std::endl
	    << "Click to set a second circle." << std::endl;
  int keycode = 0;
  while(keycode != 27) {

    if(info) {
      image = cv::Scalar(255, 255, 255);

      dubins::draw(image, frame, C, ARG_COLOR, 1);
      dubins::draw(image, frame, M, ARG_COLOR, 1);
      cv::imshow("image",image);
    }
    keycode = cv::waitKey(100) & 0xFF;
  }
  
  return 0;
}

  
