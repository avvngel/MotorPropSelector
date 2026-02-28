#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>

int main(){
  auto mask = cv::Mat_<std::uint8_t>({
    7, 7, 7, 7, 7, 
    7, 8, 7, 7, 7, 
    7, 7, 9, 7, 7, 
    7, 7, 7, 10, 7, 
    7, 7, 7, 7, 11 
  });

  mask = mask.reshape(5, 5);

  auto itr = cv::LineIterator(mask, cv::Point(0, 0), cv::Point(4, 4));

  const int count = itr.count;
  std::cout << "count: " << count << std::endl;
  
  for (int i{}; i != count; ++i){
    cv::Point pt = itr.pos();
    std::uint8_t val = *(*itr);
    std::cout << "x: " << pt.x 
              << ", y: " << pt.y
              << ", \t val: " << static_cast<int>(val) << std::endl;
    ++itr;
  }
  return 0;
}

