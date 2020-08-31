#include <opencv2/opencv.hpp>
#include <demo2d.hpp>
#include <dubins.hpp>



#define RHO .75
#define DIRECTION_COLOR  cv::Scalar(100, 100, 255)
#define CIRCLE_COLOR     cv::Scalar( 50,  50,  50)
#define TANGENT_COLOR    cv::Scalar(200, 150, 100)

class OnMouseInfo {
private:
  cv::Mat image;
  const demo2d::opencv::Frame& frame;
  dubins::Arc aleft;
  dubins::Arc aright;

  mutable bool changed = true;
  
  void update_arcs() {
    switch(left) {
    case dubins::Direction::Clockwise:
      aleft.theta_end = -dubins::to_rad(60);
      break;
    case dubins::Direction::CounterClockwise:
      aleft.theta_end = +dubins::to_rad(60);
      break;
    }
    switch(right) {
    case dubins::Direction::Clockwise:
      aright.theta_end = -dubins::to_rad(60);
      break;
    case dubins::Direction::CounterClockwise:
      aright.theta_end = +dubins::to_rad(60);
      break;
    }
  }
  
public:
  dubins::Direction left  = dubins::Direction::Clockwise;
  dubins::Direction right = dubins::Direction::Clockwise;
  

  OnMouseInfo(cv::Mat image, const demo2d::opencv::Frame& frame,
	    const dubins::Circle& cleft, const dubins::Circle& cright)
    : image(image), frame(frame),
      aleft(cleft.O, cleft.radius*.9, 0, 0),
      aright(cright.O, cright.radius*.9, 0, 0){
    update_arcs();
  }

  operator bool() const {bool res = changed; changed = false; return res;}

  void mouse_at(int x, int y) {
    auto M = frame(cv::Point(x, y));
    if(demo2d::d2(M, aleft.C.O) < demo2d::d2(M, aright.C.O))
      left = !left;
    else
      right = !right;
    update_arcs();
    changed = true;
  }

  void draw() {
    {
      dubins::Arc a {aleft.C, aleft.theta_end*.9, aleft.theta_end};
      dubins::draw(image, frame,  aleft, DIRECTION_COLOR, 1);
      dubins::draw(image, frame,      a, DIRECTION_COLOR, 5);
    }
    {
      dubins::Arc a {aright.C, aright.theta_end*.9, aright.theta_end};
      dubins::draw(image, frame,  aright, DIRECTION_COLOR, 1);
      dubins::draw(image, frame,       a, DIRECTION_COLOR, 5);
    }
  }
};

void on_mouse(int event, int x, int y, int, void* user_data) {
  auto& info = *(reinterpret_cast<OnMouseInfo*>(user_data));
  if(event == cv::EVENT_LBUTTONDOWN)
    info.mouse_at(x, y);
}

int main(int argc, char* argv[]) {
  
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  auto image = cv::Mat(400, 800, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .4*image.size().height, true);

  dubins::Circle c_left  {-1, 0, RHO};
  dubins::Circle c_right { 1, 0, RHO};

  OnMouseInfo info(image, frame, c_left, c_right);
  cv::setMouseCallback("image", on_mouse, reinterpret_cast<void*>(&info));

  std::cout << std::endl
	    << "Press ESC to quit." << std::endl
	    << "click on the circles in order to toggle their direction." << std::endl
	    << std::endl;
  int keycode = 0;
  while(keycode != 27) {

    if(info) {
      image = cv::Scalar(255, 255, 255);
      
      dubins::draw(image, frame,  c_left, CIRCLE_COLOR, 1);
      dubins::draw(image, frame, c_right, CIRCLE_COLOR, 1);
      info.draw();

      // Let us compute and draw the tangent.
      if(auto tangent = dubins::tangent(c_left.O,  info.left,
					c_right.O, info.right,
					RHO); tangent)
	cv::line(image, frame(tangent->first), frame(tangent->second), TANGENT_COLOR, 3);
      
      // Let us display the result.
      cv::imshow("image",image);
    }
    keycode = cv::waitKey(100) & 0xFF;
  }
  
  return 0;
}
