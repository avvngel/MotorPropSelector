#pragma once

namespace img {

using PosPxI = geom::Pos<PxI>;
using PosPxD = geom::Pos<PxD>;


using CvPosPx = cv::Point;

template <unit::IsUnit U>
using CvPosPx = cv::Point_<U::RepT>;

using CvPosPxI = CvPosPx<PxI>;
using CvPosPxD = CvPosPx<PxD>;

using PosPxIVec = std::vector<PosPxI>;
using PosPxDVec = std::vector<PosPxD>;

template <unit::IsUnit U>
CvPosPx<U> to_cv(Pos<U> in){
  using geom::x;
  using geom::y;

  return CvPosPx {
    .x = x(in),
    .y = y(in)
  };
}

} // namespace img
