#pragma once

#include <cmath>

namespace unit {

template <IsFPUnit U>
U fmin(U left, U right){
  return { std::fmin(left.val, right.val) };
}

template <IsFPUnit U>
U max(U left, U right){
  return { std::max(left.val, right.val) };
}

template <IsFPUnit U>
U hypot(U x, U y){
  return { std::hypot(x.val, y.val) };
}


} // namespace unit

