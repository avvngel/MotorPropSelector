#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>


int main(){

  cv::Mat_<double> tmp{
    7, 7, 7, 7, 7,
    7, 7, 7, 7, 7,
    7, 7, 7, 7, 7,
    7, 7, 7, 7, 7,
    7, 7, 7, 7, 7
  };
  tmp = tmp.reshape(1, 5);
  cv::Mat_<double> tmp_fake_copy = tmp;
  auto tmp_real_copy = tmp.clone();
  auto maybe_copy_1 = cv::Mat_<int>(tmp);
  cv::Mat_<int> maybe_copy_2 = tmp;

  tmp_fake_copy.at<double>(cv::Point(1, 1)) = 1000;
  
  std::cout << tmp << std::endl;
  std::cout << tmp_fake_copy << std::endl;
  std::cout << tmp_real_copy << std::endl;
  std::cout << maybe_copy_1 << std::endl;
  std::cout << maybe_copy_2 << std::endl;

  return 0;
}
