#include <opencv2/opencv.hpp>
#include <demo2d.hpp>
#include <dubins.hpp>

#define RHO .5
#define DIR_COLOR cv::Scalar(175, 175, 175)
#define ARG_COLOR cv::Scalar(  0,   0,   0)
#define CLK_COLOR cv::Scalar(175,  50,  50)
#define TRI_COLOR cv::Scalar( 50,  50, 175)

class OnMouseInfo {
public:
  
  demo2d::Point P;
  
private:
  const demo2d::opencv::Frame& frame;

  mutable bool changed = true;

public:
  OnMouseInfo(const demo2d::opencv::Frame& frame) : P(), frame(frame) {}
  

  operator bool() const {bool res = changed; changed = false; return res;}
  
  void mouse_at(int x, int y) {
    P = frame(cv::Point(x, y));
    changed = true;
  }
};

void on_mouse(int event, int x, int y, int, void* user_data) {
  auto& info = *(reinterpret_cast<OnMouseInfo*>(user_data));
  info.mouse_at(x, y);
}

void draw_oriented_arc(cv::Mat& image, const demo2d::opencv::Frame& frame,
		       const dubins::Arc& arc,
		       const cv::Scalar& color) {
  dubins::draw(image, frame, arc, color, 3);
  dubins::Arc arrow {arc};
  arrow.C.radius = .75*RHO;
  dubins::draw(image, frame, arrow, DIR_COLOR, 1);
  arrow.theta_start = arrow.theta_end - dubins::to_rad(10);
  dubins::draw(image, frame, arrow, DIR_COLOR, 3);
}

int main(int argc, char* argv[]) {
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  auto image = cv::Mat(600, 600, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .2*image.size().width, true);

  OnMouseInfo info(frame);
  cv::setMouseCallback("image", on_mouse, reinterpret_cast<void*>(&info));

  demo2d::Point  O {0., 0.}; // Origin
  dubins::Circle C {O, RHO};
  dubins::Circle M {C};

  
  std::cout << std::endl
	    << "Press ESC to quit." << std::endl
	    << "Click to set a second circle." << std::endl;
  int keycode = 0;
  while(keycode != 27) {

    if(info) {
      M.O = info.P;
      image = cv::Scalar(255, 255, 255);

      if(auto arc = dubins::tangent(C.O, M.O, dubins::Direction::Clockwise, RHO); arc)
	draw_oriented_arc(image, frame, *arc, CLK_COLOR);
      if(auto arc = dubins::tangent(C.O, M.O, dubins::Direction::CounterClockwise, RHO); arc) 
	draw_oriented_arc(image, frame, *arc, TRI_COLOR);

      dubins::draw(image, frame, C, ARG_COLOR, 3);
      dubins::draw(image, frame, M, ARG_COLOR, 3);
      cv::imshow("image",image);
    }
    keycode = cv::waitKey(10) & 0xFF;
  }
  
  return 0;
}

  
