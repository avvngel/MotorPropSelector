# pragma once

#include <cmath>

struct TrigInfo{
  Radian theta;
  double cos;
  double sin;
}

TrigInfo get_trig_info(Radian theta){
  return TrigInfo{
    .theta = theta,
    .cos = std::cos(theta.val),
    .sin = std::sin(theta.val)
  }
}
