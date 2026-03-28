# pragma once

namespace img::scan {


inline cv::Mat local_occupancy_mask(
  const cv::Mat& mask_in,
  int kernel_radius,
  double occupancy_thresh
){
  int kernel_dim = 2*kernel_radius + 1;
  auto tmp = cv::Mat_<float>(mask_in.size());

  cv::boxed_filter(
    mask_in, 
    tmp, 
    tmp.depth(),
    cv::Size(kernel_dim, kernel_dim),
    cv::Point(-1, -1),
    true,
    cv::BORDER_CONSTANT
  );
  cv::Mat mask_out(mask_in.size(), mask_in.type());

  cv::threshold(
    tmp,
    mask_out,
    occupancy_thresh,
    kMaxMaskByteVal,
    cv::THRESH_BINARY
  );
  return mask_out;
}


} // namespace scan
