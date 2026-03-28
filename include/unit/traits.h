#pragma once

#include "unit.h"

namespace unit {

template <class Unit>
constexpr typename Unit::RepT kMaxRep = std::numeric_limits<
  typename Unit::RepT
>::max();

template <class Unit>
constexpr Unit kMaxValue = Unit( kMaxRep<Unit> );


// namespace unit


