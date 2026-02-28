#include <iostream>
#include <opencv2/core/types.hpp>

int main(){

  cv::Point_<double> pt_1(3.4, 3.5);
  cv::Point pt_2(3, 3);
  cv::Point pt_3 = pt_1;

  std::cout << "pt_1 type: " << typeid(pt_1.x).name() << std::endl;
  std::cout << "pt_2 type: " << typeid(pt_2.x).name() << std::endl;
  std::cout << "pt_3 type: " << typeid(pt_3.x).name() << std::endl;
  std::cout << "pt_3.x: " << static_cast<int>(pt_3.x) << std::endl;
  std::cout << "pt_3.y: " << static_cast<int>(pt_3.y) << std::endl;


  return 0;
}
