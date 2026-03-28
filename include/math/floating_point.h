# pragma once

#include <cmath>

namespace math {



constexpr bool effectively_zero(double num, double scale){
  double scale_ = std::max(1.0, std::abs(scale));
  return std::abs(num) < kEps*scale_;
}



} // namespace math

