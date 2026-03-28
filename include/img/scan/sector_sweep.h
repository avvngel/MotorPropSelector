# pragma once
namespace scan {

template <Aggregator A, TermPolicy T>
ScanRes sector_sweep_by_rays(
  A& radial_agg,
  const cv::Mat& img,
  cv::Point origin,
  size_t n_samples = 360,
  PolarRect p_rect = kFullImg,
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4
  T& radial_term = NoTerm{}
){
  // Returns first hit

  AngularRange theta = p_rect.theta;
  Radian delta_theta = (theta.end - theta.start)
                       / static_cast<double>(n_samples);
  Radian curr_theta = theta.start;

  ScanRes = res{ .found = false, .pt = origin };
  for (int i{}; i != n_samples; ++i){
    res = radial_scan(
      radial_agg, 
      img, 
      origin, 
      curr_theta, 
      p_rect.r, 
      kernel_radius,
      kernel_occupancy_thresh,
      radial_term
    );

    if (res.found){
      break;
    }
    curr_theta += delta_theta;
  }

  return res;
}

template <
  ScanAgg S, 
  PxAgg P, 
  TermPolicy ST, 
  TermPolicy RT
>
ScanRes sector_sweep_by_rings(
  S& sector_agg,
  P& radial_agg,
  const cv::Mat& img,
  cv::Point origin,
  size_t n_samples = 360,
  PolarRect p_rect = kFullImg,
  ST& sector_term = TermOnHit{}
  RT& radial_term = TermOnHit{}
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4
){
  // Returns first hit

  RadialRange theta = p_rect.theta;
  Radian delta_theta = (theta.end - theta.start)
                       / static_cast<double>(n_samples);
  Radian curr_theta = theta.start;

  ScanRes = res{ .found = false, .pt = origin };
  for (int i{}; i != n_samples; ++i){
    res = radial_scan(
      radial_agg, 
      img, 
      origin, 
      curr_theta, 
      p_rect.r, 
      kernel_radius,
      kernel_occupancy_thresh,
      radial_term
    );

    res = sector_agg(res);

    if (sector_term(res)){
      break;
    }
    curr_theta += delta_theta;
  }

  return res;
}


} // namespace scan
