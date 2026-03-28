# pragma once

#include <cstdint>

namespace img {

struct Mask8 { 
  std::uint8_t val; 

  constexpr explicit Mask8(std::uint8_t val_) : val(val_) {}
};

inline constexpr Mask8 kMaxMaskByteVal{255}; 
inline constexpr Mask8 kBladePxValue{kMaxMaskByteVal}; 
inline constexpr Mask8 kBackgroundPxValue{0}; 


using Mask8s = std::vector<Mask8>;


class Mask {

public:

  explicit Mask(cv::Mat m) : mat_( std::move(m) ) {
    CV_Assert(mat_.type() == CV_8UC1);
  }

  const cv::Mat& mat(){
    return mat_;
  }

  bool empty(){
    return mat_.empty();
  }

  int cols(){
    return mat_.cols;
  }

  int rows(){
    return mat_.rows;
  }

private:
  cv::Mat mat_;

};


} // namespace img
