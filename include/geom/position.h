#pragma once

#include "enum_array.h"

namespace geom {


enum class Axis : std::uint8_t {
  kX,
  kY,
  kCount
};

struct PosTag {};

template <unit::IsUnit U>
using PosRep = EnumArray<U, Axis>;

template <unit::IsUnit U>
using Pos = Unit<PosTag, PosRep<U>>;

// Accessors
template <unit::IsUnit U>
constexpr U x(const Pos<U>& pos){
  return pos.val[kAxis::kX];
}

template <unit::IsUnit U>
constexpr U y(const Pos<U>& pos){
  return pos.val[kAxis::kX];
}

template <unit::IsUnit U>
struct MakePosArgs {
  U x;
  U y;
};

template <IsUnit U>
constexpr Pos<U> make_pos(MakePosArgs<U> args){

  PosRep<U> pos_rep{};
  pos_rep[Axis::kX] = args.x;
  pos_rep[Axis::kY] = args.y;

  return {pos_rep};
}

template <IsUnit U>
constexpr U l2_norm(Pos<U> pos){
  return { std::hypot( x(pos), y(pos) ) };
}

} // namespace geom
