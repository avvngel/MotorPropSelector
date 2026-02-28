# pragma once

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cmath>
#include <numeric>
#include <utility>
#include <iterator>
#include "units.h"
#include "chord.h"

inline constexpr auto kInfDbl = std::numeric_limits<double>::infinity();
inline constexpr auto kInfInt = std::numeric_limits<int>::infinity();
inline constexpr uint8_t kMaxMaskByteVal = 255; 
inline constexpr uint8_t kBladePxValue = kMaxMaskByteVal; 
inline constexpr uint8_t kBackgroundPxValue = 0; 
inline constexpr double kEpsScale = 64.0;
inline constexpr double kEps = kEpsScale*std::numeric_limits<double>::epsilon();
inline constexpr double kPi = std::numbers::pi;
inline constexpr Radian kFullTurnRads = Radian{ 2.0*kPi };

struct MaskByte { std::uint8_t val; };

constexpr bool effectively_zero(double num, double scale){
  double scale_ = std::max(1.0, std::abs(scale));
  return std::abs(num) < kEps*scale_;
}

constexpr cv::Point_<double> polar_to_rect(
  double r, 
  const TrigInfo theta_trig,
  cv::Point origin = cv::Point{.x = 0, .y = 0}
){
  return cv::Point_<double>{
    .x = origin.x + r*theta_trig.cos,
    .y = origin.y + r*theta_trig.sin
  };  
}

constexpr cv::Point_<double> polar_to_rect(
  double r, 
  Radian theta,
  cv::Point origin = cv::Point{.x = 0, .y = 0}
){
  TrigInfo trig_theta = get_trig_info(theta);
  return polar_to_rect(
    r, 
    trig_theta,
    origin
  );
}

struct ScanRes{ 
  bool found;
  cv::Point pt;
};

template <class T>
concept ByteScanner = requires (T a, MaskByte b){
  { a(b) } -> std::same_as_v<bool>;
}


struct IsBladePx { 
  constexpr bool operator(MaskByte px){
    return px == kBladePxValue;
  }
};

struct IsBackgroundPx {
  constexpr bool operator(MaskByte px){
    return px == kBackgroundPxValue;
  }
}

struct IsTransitionPx { 
  std::uint8_t comp_value;

  constexpr bool operator(MaskByte px){
    return px == comp_value;
  }
};

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

template <ByteScanner F>
ScanRes line_scan(
  F& scan_func,
  const cv::Mat& mask,
  cv::Point start_pt,
  cv::Point end_pt,
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4
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
    cv::Point pt = itr.pos();
    uint8_t px_val = *(*itr);

    if (scan_func(px_val)){
      res.found = true;
      res.pt = pt;
      break;
    } 
    ++itr;
  }

  return res;
}

inline double max_radius(
  const cv::Mat& img,
  const cv::Point origin,
  const TrigInfo theta_trig
){

  CV_Assert(0 <= origin.x && origin.x < img.cols);
  CV_Assert(0 <= origin.y && origin.y < img.rows);

  const bool facing_right = !std::signbit(theta_trig.cos);
  const bool facing_down  = !std::signbit(theta_trig.sin);

  const double v_wall_x = (facing_right) ? img.cols - 1 : 0;
  const double h_wall_y = (facing_down)  ? img.rows - 1 : 0;

  const double x_disp_from_v_wall = v_wall_x - origin.x;
  const double y_disp_from_h_wall = h_wall_y - origin.y;
  
  double r_from_x = (!effectively_zero(theta_trig.cos, x_disp_from_v_wall)) 
    ? x_disp_from_v_wall/theta_trig.cos
    : kInfDbl;
  double r_from_y = (!effectively_zero(theta_trig.sin, y_disp_from_h_wall)) 
    ? y_disp_from_h_wall/theta_trig.sin
    : kInfDbl; 

  return std::fmin(r_from_x, r_from_y);
}

inline double max_radius(
  const cv::Mat& img,
  const cv::Point origin,
  Radian theta
){
  TrigInfo trig_theta = get_trig_info(theta);
  return max_radius(
    img,
    origin,
    trig_theta
  );
}


template <ByteScanner F>
inline ScanRes radial_scan(
  F& scan_func,
  const cv::Mat& img,
  cv::Point origin,
  Radian theta,
  double radius_start = 0.0,
  double radius_end = kInfDbl,
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4
){
  TrigInfo theta_trig = get_trig_info(theta);
  double max_r = max_radius(
    img, 
    origin, 
    theta_trig
  );

  radius_start = std::min(
    radius_start, max_r
  );

  cv::Point start_pt = polar_to_rect(
    radius_start,
    theta_trig,
    origin
  );

  radius_end = std::min(
    radius_end, max_r
  );

  cv::Point end_pt = polar_to_rect(
    radius_end,
    theta_trig,
    origin
  );
  return line_scan(
    scan_func,
    img, 
    start_pt, 
    end_pt, 
    kernel_radius,
    kernel_occupancy_thresh
  );
}

template <ByteScanner F>
ScanRes azimuthal_scan(
  F& scan_func,
  const cv::Mat& mask,
  cv::Point origin,
  size_t n_samples,
  double radius,
  Radian theta_start = Radian{0.0},
  Radian theta_end = kFullTurnRads,
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4
){
  CV_Assert(mask.type() = CV_8UC1);

  Radian delta_theta = (theta_end - theta_start)
    / static_cast<double>(n_samples);
  Radian curr_theta = theta_start;

  unsigned char* data = mask.data;
  size_t row_step = mask.step[0];
  size_t col_step = mask.step[1];

  for (int i{}; i != n_samples; ++i){

    TrigInfo curr_theta_trig = get_trig_info(curr_theta);
    double curr_radius = std::min(
        radius, max_radius(mask, origin, curr_theta_trig)
    );

    cv::Point pt = polar_to_rect(
      curr_radius,
      curr_theta_trig,
      origin
    );
    
    std::uint8_t px = *(data + row_step*pt.x + col_step*pt.y);

    if (scan_func(px)){ 
      return ScanRes{ .found = true, .pt = pt }; 
    }
    curr_theta += delta_theta;
  }
  return ScanRes{ .found = false, .pt = origin };
}

// TermPolicy takes a ScanRes and returns a bool
// that decides to term or not.
// examples include: FindFirst, FindLast, NoTerm
//
// ByteScanners can have state. So we can have a 
// sector scan that terminates when a ray has NO
// hits or ALL hits by setting the TermPolicy on
// the inner scan (radial or azimuthal) to FindLast
// and keeping a member variable in the ByteScanner
// that tracks if there has been ANY hit/miss
// and returns the appropriate bool

template <ByteScanner F, TermPolicy T>
ScanRes sector_scan(
  F& scan_func,
  const cv::Mat& img,
  cv::Point origin,
  T& term_func,
  size_t n_samples = 360,
  Radian theta_start = Radian{0.0},
  Radian theta_end = kFullTurnRads,
  double inner_radius = 0,
  double outer_radius = kInfDbl,
  int kernel_radius = 1,
  double kernel_occupancy_thresh = 0.4
){
  // Terminates on first scan_func hit 

  Radian delta_theta = (theta_end - theta_start)
                       / static_cast<double>(n_samples);
  Radian curr_theta = theta_start;

  ScanRes = res{ .found = false, .pt = origin };
  for (int i{}; i != n_samples; ++i){
    ScanRes curr_res = radial_scan(
      scan_func, 
      img, 
      origin, 
      curr_theta, 
      inner_radius, 
      outer_radius,
      kernel_radius,
      kernel_occupancy_thresh
    );

    if (curr_res.found){
      return curr_res;
    }

    curr_theta += delta_theta;

  }

}


struct ThetaBand { Radian lo, hi; };

ThetaBand bound_blade(
  const cv::Mat& mask,
  cv::Point origin,
  double blade_root_r_px,
  int fine_search_N = 10000
  int coarse_search_N = 8
){
  ThetaBand res{Radian{0.0}, Radian{0.0}};
  ScanRes coarse_search_res = sector_scan(
    IsBladePx{}, 
    mask, 
    origin,
    coarse_search_N,
    Radian{0.0},
    kFullTurnRads,
    blade_root_r_px
  );

  if (!coarse_search_res.found){
    return res;
  }

  Radian incident_theta = rect_to_polar(
    coarse_search_res.pt, 
    origin
  ).theta;

  ScanRes fine_search_cw = sector_scan(
    IsBackgroundPx{}, 
    mask, 
    origin,
    fine_search_N,
    incident_theta,
    incident_theta + kFullTurnRads,
    blade_root_r_px
  );

  ScanRes fine_search_ccw = sector_scan(
    IsBackgroundPx{}, 
    mask, 
    origin,
    fine_search_N,
    incident_theta - kFullTurnRads,
    kFullTurnRads,
    blade_root_r_px
  );

  res.lo = rect_to_polar(
    fine_search_ccw.pt, 
    origin 
  ).theta;

  res.hi = rect_to_polar(
    fine_search_cw.pt, 
    origin 
  ).theta;

  return res;

}


ChordDist mask_to_chord_dist(
  const std::string& mask_filepath,
  cv::Point center,
  double ref_pitch_in,
  double ref_diameter_in,
  double blade_root_r_px,
  size_t num_stations,
){

  ChordDist dist{};
  dist.reserve(num_stations);
  
  cv::Mat mask = cv::imread(mask_filepath, cv::IMREAD_GRAYSCALE);
  if (mask.empty()) { return dist; } 

  ThetaBand blade_azimuthal_bounds = bound_blade(
    mask,
    center,
    blade_root_r_px
  );

  
  

  





  return dist;
}
