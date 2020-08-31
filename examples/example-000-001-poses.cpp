#include <opencv2/opencv.hpp>
#include <demo2d.hpp>
#include <dubins.hpp>

#include <iostream>
#include <algorithm>
#include <vector>
#include <iterator>

#define NB 13
#define RADIUS 1

int main(int argc, char* argv[]) {
  
  auto image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255,255,255));
  auto frame = demo2d::opencv::direct_orthonormal_frame(image.size(), .1*image.size().width, true);

  
  for(unsigned int i = 0; i < NB; ++i) {
    double theta = 2*dubins_PI*i/NB;
    dubins::draw(image, frame,
		 {RADIUS*std::cos(theta), RADIUS*std::sin(theta), theta + 1},
		 7, .5, cv::Scalar(0, 0, 150), 3);
  }

  
  std::vector<dubins::Pose> poses;
  auto out = std::back_inserter(poses);
  for(unsigned int i = 0; i < 2*NB; ++i) {
    double theta = dubins_PI*i/NB;
    *(out++) = {2*RADIUS*std::cos(theta), 2*RADIUS*std::sin(theta), theta - 1};
  }

  auto pd = dubins::pose_drawer<dubins::Pose>(image, frame,
					      [](const auto&     ) {return true;},
					      [](const auto& pose) {return pose;},
					      [](const auto&     ) {return 7;},
					      [](const auto&     ) {return .5;},
					      [](const auto&     ) {return cv::Scalar(150, 0, 0);},
					      [](const auto&     ) {return 3;});

  // We draw the content of the vector.
  std::copy(poses.begin(), poses.end(), pd);
  

  // Let us display the result.
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  cv::imshow ("image",      image);
  cv::waitKey(0);

  
  return 0;
}
