# pragma once 

namespace img::scan {



template <Aggregator A, TermPolicy T>
ScanRes line_scan(
  A aggregator,
  const cv::Mat& mask,
  cv::Point start_pt,
  cv::Point end_pt,
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4
  T& term = TermOnHit{},
){
  CV_Assert(mask.type() = CV_8UC1);

  cv::Mat robust_mask{};

  if (kernel_radius){
    robust_mask = local_occupancy_mask(
      mask
      kernel_radius,
      kernel_occupancy_thresh
    );
  }
  robust_mask = mask;

  auto itr = cv::LineIterator(
    robust_mask, 
    start_pt, 
    end_pt
  );

  ScanRes res{ .found = false, .pt = end_pt };
  const int count = itr.count;

  for (int i{}; i < count; ++i){
    
    uint8_t px = *(*itr);
    
    res.found = aggregator(px, pt);
    res.pt = itr.pos();;

    if (term(res)){
      break;
    }
    ++itr;
  }

  return res;
}


template <Aggregator A, TermPolicy T>
inline ScanRes radial_scan(
  A& aggregator,
  const cv::Mat& img,
  cv::Point origin,
  Radian theta,
  RadialRange radius = kMaxRadialRange,
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4,
  T& term_func = TermFirstHit{},
){
  TrigInfo theta_trig = get_trig_info(theta);
  double max_r = max_radius(
    img,
    origin,
    theta_trig
  );

  radius.start = std::min(
    radius.start, max_r
  );

  cv::Point start_pt = polar_to_rect(
    radius.start,
    theta_trig,
    origin
  );

  radius.end = std::min(
    radius.end, max_r
  );

  cv::Point end_pt = polar_to_rect(
    radius.end,
    theta_trig,
    origin
  );
  return line_scan(
    aggregator,
    img,
    start_pt,
    end_pt,
    kernel_radius,
    kernel_occupancy_thresh,
    term_func
  );
}

} // namespace img::scan
