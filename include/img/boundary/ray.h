# pragma once

#include "unit/unit.h"

namespace img::boundary {


inline PxD ray_len_to_edge(
  const Mask& img,
  const PosPxD origin,
  const TrigInfo theta_trig
){

  using math::effectively_zero;
  using geom::x;
  using geom::y;

  PxD o_x = x(origin);
  PxD o_y = y(origin);

  CV_Assert(PxD{0} <= o_x && o_x < PxD{ img.cols() });
  CV_Assert(PxD{0} <= o_y && o_y < PxD{ img.rows() });

  const bool facing_right = !std::signbit(theta_trig.cos);
  const bool facing_down  = !std::signbit(theta_trig.sin);

  const PxD v_wall_x{ (facing_right) ? img.cols() - 1 : 0 };
  const PxD h_wall_y{ (facing_down)  ? img.rows() - 1 : 0 };

  const PxD x_disp_from_v_wall = v_wall_x - o_x;
  const PxD y_disp_from_h_wall = h_wall_y - o_y;
  
  PxD r_from_x{ 
    (!effectively_zero(theta_trig.cos, x_disp_from_v_wall)) 
    ? x_disp_from_v_wall/theta_trig.cos
    : kInfDbl
  };

  PxD r_from_y{
    (!effectively_zero(theta_trig.sin, y_disp_from_h_wall)) 
    ? y_disp_from_h_wall/theta_trig.sin
    : kInfDbl
  };

  return unit::fmin(r_from_x, r_from_y);
}


inline PxD ray_len_to_edge(
  const Mask& img,
  const PosPxD origin,
  Radian theta
){
  TrigInfo trig_theta = get_trig_info(theta);
  return ray_len_to_edge(
    img,
    origin,
    trig_theta
  );
}


inline PxD ray_len_to_edge(
  const Mask& img,
  const PosPxD origin
){

  const PxD o_x{ x(origin) };
  const PxD o_y{ y(origin) };
  
  CV_Assert( PxD{0} <= o_x && o_x < PxD{img.cols()} );
  CV_Assert( PxD{0} <= o_y && o_y < PxD{img.rows()} );

  const PxD v_wall_max_x = unit::max(o_x, PxD{img.cols - 1});
  const PxD h_wall_max_y = unit::max(o_y, PxD{img.rows - 1});

  return unit::hypot(v_wall_max_x, h_wall_max_y);
}




} // namespace img::boundary
