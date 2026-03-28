# pragma once

#include "geom/trig.h"
#include "geom/polar.h"

namespace scan {

  inline double get_blade_radius(
    const Mask& mask,
    cv::Point center,
    double blade_root_r_px,
    geom::AngularRange angular_bounds
  ){
    using geom::trig::bisect_shortest_arc;

    Radian mid_angle = bisect_shortest_arc(angular_bounds);
    TrigInfo trig = get_trig_info(mid_angle);
    double mid_ray_len = max_radius(mask, center, trig);

    double initial_scan_N = 500.0;
    
    ScanRes mid_ray_scan = ring_scan(
      IsBladePx{}, 
      mask, 
      center,
      mid_ray_len,
      initial_scan_N,
      angular_bounds
    );

    PolarRect search_area{
      .theta = angular_bounds,
      .r = RadialRange{
        .start = mid_ray_len,
        .end = mid_ray_len
      }
    };

    double refine_scan_N = 500.0;

    ScanRes refine_scan_res{};
    if (mid_ray_scan.found){
      search_area.end = max_radius(mask, center);
      refine_scan_res = sector_sweep_by_rings(
        AllHits<IsBackgroundPx>{},
        mask,
        center,
        refine_scan_N,
        search_area
      );
    } else {
      search_area.end = blade_root_r_px;
      refine_scan_res = sector_sweep_by_rings(
        IsBladePx{},
        mask,
        center,
        refine_scan_N,
        search_area
      );
    }
    double blade_radius = (refine_scan_res.found) 
      ? cv::norm(refine_scan_res.pt - center)
      : blade_root_r_px;

    return blade_radius;
  }


} // namespace scan
