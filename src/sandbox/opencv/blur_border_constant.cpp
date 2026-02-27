#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>

int main(){

  auto mask = (cv::Mat_<std::uint8_t>({
    1, 0, 1, 
    1, 1, 1, 
    1, 1, 1 
  }));//.reshape(1,3);
  mask = mask.reshape(1, 3);
  std::cout << mask << std::endl;
  std::cout << (mask.type() == CV_8UC1) << std::endl;
  
  auto blurred_mask = cv::Mat_<double>();
  cv::boxFilter(
    mask,
    blurred_mask,
    blurred_mask.depth(),
    cv::Size_(3, 3),
    cv::Point(-1, -1),
    true,
    cv::BORDER_CONSTANT
  );
  std::cout << blurred_mask << std::endl;
  std::cout << blurred_mask.type() << std::endl;

  cv::threshold(
    blurred_mask,
    mask,
    .5,
    255.0,
    cv::THRESH_BINARY
  );
  

  std::cout << mask << std::endl;
  std::cout << (mask.type()) << std::endl;

  return 0;
}
