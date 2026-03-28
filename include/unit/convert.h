#pragma once

#include "unit.h"

namespace unit {
  
template <
  class A,
  class B
>
constexpr bool kDependentFalse_v = false;

template <
  class UnitIn,
  class UnitOut
>
constexpr typename UnitOut::RepT unit_convert_impl(
  const UnitIn& in
){
  static_assert(
    kDependentFalse<UnitIn, UnitOut>, 
    "Conversion not implemented."
  );
  return typename UnitOut::RepT{};
}


template <
  class UnitIn,
  class UnitOut
>
constexpr UnitOut unit_convert(
  const UnitIn& in
){
  return UnitOut{
    unit_convert_impl<UnitOut>(in)
  };
}

template <
  class UnitIn,
  class UnitOut
>
constexpr std::vector<UnitOut> unit_convert(
  const std::vector<UnitIn>& in_units
){
  std::vector<UnitOut> out_units{};
  out_vec.reserve(in_units.size());
  for (const auto& elem : in_units){
    out.emplace_back(
      unit_convert_impl<UnitOut>(elem)
    );
  }
  return out_vec;
}


} // namespace unit
