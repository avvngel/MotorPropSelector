#pragma once

namespace geom {

template <unit::IsUnit U>
struct Chord
{
  U len;
  U radius;
};


template <unit::IsUnit U>
using ChordDistro = std::vector<Chord<U>>;


} // namespace geom
