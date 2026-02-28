#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>

int main(){

  cv::Mat_<double> tmp{
    1, 1, 2, 3, 4,
    5, 6, 7, 8, 9,
    10, 11, 12, 13, 14,
    15, 16, 17, 18, 19,
    20, 21, 22, 23, 24
  };
  tmp = tmp.reshape(1, 5);
  unsigned char* data = tmp.data;
  size_t row_step = tmp.step[0];
  size_t col_step = tmp.step[1];

  double px_2_2 = *reinterpret_cast<double*>(data + row_step*2 + col_step*2);
  std::cout << tmp << std::endl;
  std::cout << static_cast<int>(*data) << std::endl;
  std::cout << static_cast<int>(*(data+col_step)) << std::endl;
  std::cout << row_step << std::endl;
  std::cout << col_step << std::endl;
  std::cout << static_cast<int>(px_2_2) << std::endl;
  


  return 0;
}
