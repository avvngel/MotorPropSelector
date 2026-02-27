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
inline constexpr double kEpsScale = 64.0;
inline constexpr double kEps = kEpsScale*std::numeric_limits<double>::epsilon();
inline constexpr double kPi = std::numbers::pi;
inline constexpr double kFullTurnRads = 2.0*kPi;

struct MaskByte { std::uint8_t val; };

constexpr bool effectively_zero(double num, double scale){
  double scale_ = std::max(1.0, std::abs(scale));
  return std::abs(num) < kEps*scale_;
}

constexpr cv::Point polar_to_rect(
  double r, 
  Radian theta
  cv::Point origin = cv::Point{.x = 0, .y = 0}
){
  return cv::Point{
    .x = std::floor(origin.x + r*std::cos(theta.val)),
    .y = std::floor(origin.y + r*std::sin(theta.val))
  };
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
  double occupancy_thresh = 0.4
){
  CV_Assert(mask.type() = CV_8UC1);

  auto robust_mask = local_occupancy_mask(mask);
  ScanRes res{ .found = false, .pt = end_pt };
  auto itr = cv::LineIterator(mask, start_pt, end_pt);
  const int count = itr.count();

  for (int i{}; i < count; ++i){
    cv::Point pt = itr.pos();
    uint8_t px_val = mask.at(pt.x, pt.y);

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
  Radian theta
){

  CV_Assert(0 <= origin.x && origin.x < img.cols);
  CV_Assert(0 <= origin.y && origin.y < img.rows);

  const Radian cos_theta = std::cos(theta.val);
  const Radian sin_theta = std::sin(theta.val);

  const bool facing_right = !std::signbit(cos_theta);
  const bool facing_down  = !std::signbit(sin_theta);

  const double v_wall_x = (facing_right) ? img.cols - 1 : 0;
  const double h_wall_y = (facing_down)  ? img.rows - 1 : 0;

  const double x_disp_from_v_wall = v_wall_x - origin.x;
  const double y_disp_from_h_wall = h_wall_y - origin.y;
  
  double r_from_x = (!effectively_zero(cos_theta, x_disp_from_v_wall)) 
    ? x_disp_from_v_wall/cos_theta
    : kInfDbl;
  double r_from_y = (!effectively_zero(sin_theta, y_disp_from_h_wall)) 
    ? y_disp_from_h_wall/sin_theta
    : kInfDbl; 

  return std::fmin(r_from_x, r_from_y);
}


template <ByteScanner F>
inline ScanRes radial_scan(
  const cv::Mat& img,
  cv::Point origin,
  Radian theta,
  double radius_start = 0.0,
  F& scan_func = IsBladePx{},
  double radius_end = kInfDbl
){
  cv::Point start_pt = polar_to_rect(
    radius_start,
    theta,
    origin
  );

  radius_end = std::min(
    radius_end, 
    max_radius(img, origin, theta)
  );

  cv::Point end_pt = polar_to_rect(
    radius_end,
    theta,
    origin
  );

  return line_scan(img, start_pt, end_pt, scan_func);
  
}

template <class ResType, ByteScanner F>
ResType SectorScan(
  F& scan_func,
  const cv::Mat& img,
  cv::Point origin,
  Radian theta_start = Radian{0.0},
  Radian theta_end = Radian{kFullTurnRads},
  size_t n_samples = 360,
  double inner_radius = 0,
  double outer_radius = kInfDbl
){
  // Finds first

  Radian delta_theta = (theta_end - theta_start)
                       / static_cast<double>(n_samples);
  Radian curr = theta_start;

  for (int i{}; i != n_samples; ++i){
    radial_scan(img, origin, theta, radius_start, scan_func, )

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
  ScanRes coarse_res{.found = false, .pt = origin};
  execute_per_azimuth(radial_scan, 0.0, kFullTurnRads, coarse_search_N, coarse_res);
  
  if (!coarse_res.found){
    return res;
  }
  
  PolarCoord incident_theta = rect_to_polar(coarse_res.pt, origin);
  // use as starting theta and do fine search in both directions to find edges
  // of blade ...
  

}


ChordDist image_to_chord_dist(
  const std::string& img_filepath,
  PixCoord center,
  double ref_pitch_in,
  double ref_diameter_in,
  double blade_root_r_px,
  size_t num_stations,
){

  ChordDist dist{};
  
  cv::Mat img = cv::imread(img_filepath, cv::IMREAD_GRAYSCALE);
  if (img.empty()) { return dist; } 

  





  return dist;
}
