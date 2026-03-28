#pragma once

#include <numeric>

namespace math {

inline constexpr auto kInfDbl = std::numeric_limits<double>::infinity();
inline constexpr auto kInfInt = std::numeric_limits<int>::max();
inline constexpr double kPi = std::numbers::pi;
inline constexpr double kEpsScale = 64.0;
inline constexpr double kEps = 
  kEpsScale*std::numeric_limits<double>::epsilon();


} // namespace numeric
