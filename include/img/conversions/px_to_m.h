#pragma once

#include <cassert>
#include <vector>
#include "unit/meter.h"
#include "img/px.h"

namespace img {

struct PxPerMeter{

  constexpr explicit PxPerMeter(PxD px_dim, Meter meter_dim)
  : val(px_dim.val / meter_dim.val){
    assert(meter_dim.val > 0.0);
  } 

  double val;
};

constexpr double px_to_meter_impl(PxD px_dim, PxPerMeter scale){
  assert(px_per_m > 0.0);
  return Meter{ px_dim.val / scale.val };
}

constexpr Meter px_to_meter(PxD px_dim, PxPerMeter scale){
  return Meter(px_to_meter_impl(px_dim, scale));
}

inline ChordDist<Meter> px_to_meter(
  const ChordDist<PxD>& px_chords,
  PxPerMeter scale
){
  ChordDist<Meter> res{};
  res.reserve(px_chords.size());
  for (const auto& chord : px_chords){
    res.emplace_back({
      px_to_meter(chord.len, px_per_m),
      px_to_meter(chord.radius, px_per_m)
    });
  }
  return res;
}

} // namespace img
