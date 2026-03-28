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


ChordDistro<Meter> mask_to_chord_dist(
  const std::string& mask_filepath,
  PxPosD center,
  Inch ref_pitch,
  Inch ref_diameter,
  PxD blade_root_r,
  size_t num_stations
){
  Mask8 mask(cv::imread(mask_filepath, cv::IMREAD_GRAYSCALE));
  if (mask.empty()) { return dist; } 

  CvPxPosD cv_center{ to_cv(center) };

  PolarRect blade_p_rect = img::extract::blade_rect_from_mask(
    mask,
    cv_center,
    blade_root_r_px
  );

  ChordDistro<PxD> chord_dist_px = img::extract::chord_dist_from_rect(
    mask, 
    cv_center,
    blade_p_rect,
    num_stations
  );

  Meter ref_pitch_m{ref_pitch};
  Meter ref_diameter_m{ref_diameter};
  PxD blade_diameter_px = blade_p_rect.r.end;
  PxPerMeter scale(blade_diameter_px, ref_diameter_m)

  return px_to_meter(chord_dist_px, scale);

}

