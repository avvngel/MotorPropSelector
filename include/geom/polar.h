#pragma once

#include <cmath>
#include "units/radian.h"
#include "trig.h"
#include "unit/units.h"
#include "unit/traits.h"

namespace geom {

template <unit::IsUnit U>
struct PolarPos {
  U r;
  Radian theta;
  Pos<Unit> center;
};

template <unit::IsUnit U>
inline Pos<U> polar_to_rect(
  const U r,
  const TrigInfo theta_trig,
  const Pos<U> center
){
  return make_pos({
    .x = x(center) + r*theta_trig.cos, 
    .y = y(center) + r*theta_trig.sin
  });
}

template <unit::IsUnit U>
inline Pos<U> polar_to_rect(
  const PolarPos<U> in,
  const TrigInfo theta_trig,
){
  return polar_to_rect(
    in.r,
    theta_trig,
    in.center
  );
}

template <unit::IsUnit U>
inline Pos<U> polar_to_rect(
  const PolarPos<U> in
){
  return polar_to_rect(
    in.r,
    get_trig_info(in.theta),
    in.center
  );
}

template <unit::IsUnit U>
inline PolarPos<U> rect_to_polar(
  const Pos<U> in,
  const Pos<U> polar_center
){
  return {
    .r = l2_norm(
      in - polar_center
    ),
    .theta = std::atan2(
      static_cast<double>( y(in) ),
      static_cast<double>( x(in) )
    ),
    .center = polar_center
  };
}

struct AngularRange {
  Radian start;
  Radian end;
};

constexpr AngularRange kEmptyAngularRange{
  .start = Radian{0.0},
  .end   = Radian{0.0}
};

constexpr AngularRange kFullTurn{ 
  .start = Radian{0.0},
  .end   = kFullTurnRads
};

template <unit::IsUnit U>
struct RadialRange {
  Unit start;
  Unit end;
};

template <unit::IsUnit U>
constexpr RadialRange<U> EmptyRadialRange{
  .start = U{},
  .end   = U{}
};

template <unit::IsUnit U>
constexpr RadialRange<U> kOriginToEdge{
  .start = U{},
  .end   = kMaxValue<U>
};

template <unit::IsUnit U>
struct PolarRect {
  AngularRange theta;
  RadialRange<U> r;
};

template <unit::IsUnit U>
constexpr PolarRect kEmptyPolarDomain = PolarRect<U>{
  kEmptyAngularRange,
  kEmptyRadialRange<U>
};

template <unit::IsUnit U>
constexpr PolarRect kFullPolarDomain = PolarRect<U>{
  kFullTurn,
  kOriginToEdge<U>
};


inline Radian signed_shortest_angle_diff(Radian to, Radian from){
  Radian raw_diff{ to - from };
  TrigInfo trig = get_trig_info(raw_diff);
  return Radian{ std::atan2(trig.sin, trig.cos) };
}

inline Radian bisect_shortest_arc(Radian to, Radian from){
  return from + signed_shortest_angle_diff(to, from) / double{2.0};
}

inline Radian bisect_shortest_arc(AngularRange theta_range){
  return biesct_shortest_arc(theta_range.end, theta_range.start);
}


} // namespace geom
