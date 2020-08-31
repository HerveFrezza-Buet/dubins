#include <opencv2/opencv.hpp>
#include <demo2d.hpp>
#include <dubins.hpp>

#include <iostream>
#include <cmath>

#define ANGLE_STEP_1 .01
#define ANGLE_STEP_2 .003
#define AMPLITUDE  2

int main(int argc, char* argv[]) {
  
  auto image = cv::Mat(800, 800, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .2*image.size().width, true);

  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);

  dubins::Circle c1 {};

  std::cout << std::endl
	    << "Press ESC to quit." << std::endl
	    << std::endl;
  int keycode = 0;
  double theta = 0;
  double psi = 0;
  while(keycode != 27) {
    
    image = cv::Scalar(255, 255, 255);

    dubins::Circle c2 {demo2d::Point(std::cos(psi), std::sin(psi)) * AMPLITUDE * std::sin(theta), .9};
  
    dubins::draw(image, frame, c1, cv::Scalar(200,   0,   0), 1);
    dubins::draw(image, frame, c2, cv::Scalar(  0,   0, 200), 1);

    if(auto points = c1 && c2; points) {
      auto& [p1, p2] = *points;
      cv::circle(image, frame(p1), 5, cv::Scalar(0, 0, 0), -1);
      cv::circle(image, frame(p2), 5, cv::Scalar(0, 0, 0), -1);
    }
    
    // Let us display the result.
    cv::imshow ("image",image);
    keycode = cv::waitKey(10) & 0xFF;
    
    theta += ANGLE_STEP_1;
    if(theta > dubins_PI)
      theta -= 2*dubins_PI;
    
    psi += ANGLE_STEP_2;
    if(psi > dubins_PI)
      psi -= 2*dubins_PI;
  }

  return 0;
}
