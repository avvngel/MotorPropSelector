# pragma once

#include "px.h"

namespace img::extract {

ChordDist chord_dist(
  Mask mask,
  cv::Point hub_center,
  PolarRect blade_rect,
  size_t num_stations
){

  size_t ring_disc_n = 500; 
  
  CollectHits collect_left_endpts{};
  collect_left_endpts.hits.reserve(num_stations);

  ScanRes ignore = sector_sweep_by_rings(
    collect_left_endpts,
    IsBladePx{},
    mask,
    hub_center,
    ring_disc_n,
    blade_rect,
    NoTerm{}
  );

  CollectHits collect_right_endpts{};
  collect_right_endpts.hits.reserve(num_stations);

  ignore = sector_sweep_by_rings(
    collect_right_endpts,
    IsBladePx{},
    mask,
    hub_center,
    ring_disc_n,
    blade_rect,
    NoTerm{}
  );

  ChordDist<PxD> res{};
  res.reserve(num_stations);
  
  const PxPosIVec& left_endpts = collet_left_endpts.hits;
  const PxPosIVec& right_endpts = collet_right_endpts.hits;


  for (int i = 0; i < num_stations; ++i){
    res.emplace_back(cv::norm(left_endpts[i] - right_endpts[i]));
  }

  return res;
}


} namespace img::extract



