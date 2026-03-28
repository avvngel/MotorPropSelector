#pragma once

#include "unit/rep_cast_policy.h"

namespace detail {

  template <RepCastPolicy P>
  struct RepCaster {};

  template <>
  struct RepCaster<RepCastPolicy::kStaticCast>{

    template <
      std::integral RepIn,
      std::floating_point RepOut
    >
    static RepOut cast(RepIn in){
      return static_cast<RepOut>(in);
    }
  };


  template <>
  struct RepCaster<RepCastPolicy::kNearest>{

    template <
      std::integral RepIn,
      std::floating_point RepOut
    >
    static RepOut cast(RepIn in){
      return std::round(in);
    }
  };


  template <>
  struct RepCaster<RepCastPolicy::kFloor>{

    template <
      std::integral RepIn,
      std::floating_point RepOut
    >
    static RepOut cast(RepIn in){
      return std::floor(in);
    }
  };


  template <>
  struct RepCaster<RepCastPolicy::kCeil>{

    template <
      std::integral RepIn,
      std::floating_point RepOut
    >
    static RepOut cast(RepIn in){
      return std::ceil(in);
    }
  };


  template <>
  struct RepCaster<RepCastPolicy::kTowardZero>{

    template <
      std::integral RepIn,
      std::floating_point RepOut
    >
    static RepOut cast(RepIn in){
      return static_cast<RepOut>(in);
    }
  };

}  // detail

