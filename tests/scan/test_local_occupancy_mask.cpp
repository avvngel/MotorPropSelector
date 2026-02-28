#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

int main(){
  cv::Mat_<std::uint8_t> test_mat({
    1, 1, 0, 0, 1,
    1, 0, 1, 1, 0,
    0, 1, 0, 1, 1,
    0, 0, 0, 0, 1,
    0, 1, 0, 1, 1
  });
  double thresh = .5;
  cv::Mat_<std::uint8_t> expected({
    0, 0, 0, 0, 0,
    0, 1, 1, 1, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 1, 1,
    0, 0, 0, 0, 0
  });
  test_mat = test_mat.reshape(5, 5);
  cv::Mat actual = local_occupancy_mask(test_mat);
  if (actual != expected){
    std::cout << "Unexpected local occupancy mask." << std::endl
              << "EXPECTED: " << expected;
              << "ACTUAL: " << actual;
    return 1;
  }
  return 0;
}
