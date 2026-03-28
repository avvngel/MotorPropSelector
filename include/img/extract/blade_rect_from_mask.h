#pragma once 

namespace img::extract {

struct TieredResolution{
  int fine_bin_count = 10000;
  int coarse_bin_count = 8;
};

inline PolarRect<PxD> blade_rect_from_mask(
  const Mask& mask,
  PosPxD hub_center,
  PxD blade_root_r,
  TieredResolution resolution = {}
){
  PolarRect<PxD> res{kEmptyPolarDomain<PxD>};

  ScanRes coarse_res = ring_scan(
    IsBladePx{},
    mask,
    hub_center,
    resolution.coarse_bin_count,
    blade_root_r
  );

  Radian incident_theta = (!coarse_res.found)
    ? Radian{0.0}
    : rect_to_polar(
        coarse_search_res.pt,
        hub_center
      ).theta;

  PolarRect cw_search_area{
    .theta = AngularRange{
      .start = incident_theta,
      .end   = incident_theta + kFullTurnRads
    },
    .r = RadialRange<PxD>{
      .start = blade_root_r,
      .end   = kMaxValue<PxD>
    }
  };

  ScanRes cw_res = sector_sweep_by_rays(
    All<IsBackgroundPx>{},
    mask,
    hub_center,
    resolution.fine_bin_count,
    cw_search_area
  );

  PolarRect ccw_search_area{
    .theta = AngularRange{
        .start = incident_theta,
        .end   = incident_theta - kFullTurnRads
      },
      .r = RadialRange<PxD>{
        .start = blade_root_r,
        .end   = kMaxValue<PxD>
      }
  };

  ScanRes ccw_res = sector_scan(
    IsBackgroundPx{},
    mask,
    hub_center,
    fine_search_N,
    ccw_search_area
  );

  res.theta.start = rect_to_polar(
    cw_res.pt,
    hub_center
  ).theta;

  res.theta.end = rect_to_polar(
    ccw_res.pt,
    hub_center
  ).theta;

  res.r.start = blade_root_r;
  res.r.end = get_blade_radius(
    mask,
    hub_center,
    blade_root_r,
    res.theta
  );

  return res;
}

} // namespace img::extract
