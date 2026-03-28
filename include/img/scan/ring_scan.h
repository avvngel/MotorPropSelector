# pragma once

namespace scan {


template <Aggregator A, TermPolicy T>
ScanRes ring_scan(
  A& aggregator,
  const cv::Mat& mask,
  cv::Point origin,
  double radius,
  size_t n_samples,
  AngularRange theta = kFullTurn,
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4,
  T& term = TermOnHit{}
){
  CV_Assert(mask.type() = CV_8UC1);

  ScanRes res{ .found = false, .pt = origin }; 

  Radian delta_theta = (theta.end - theta.start)
    / static_cast<double>(n_samples);
  Radian curr_theta = theta.start;

  unsigned char* data = mask.data;
  size_t row_step = mask.step[0];
  size_t col_step = mask.step[1];

  for (int i{}; i != n_samples; ++i){

    TrigInfo curr_theta_trig = get_trig_info(curr_theta);
    
    cv::Point pt = polar_to_rect(
      radius,
      curr_theta_trig,
      origin
    );
    
    curr_theta += delta_theta;

    if !inside(mask, pt){ continue; };
    
    std::uint8_t px = *(data + row_step*pt.y + col_step*pt.x);

    res.found = aggregator(px, pt);
    res.pt = pt;

    if (term(res)){ break; }
  }
  return res;
}

} // namespace scan
