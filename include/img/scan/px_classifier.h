# pragma once

#include "mask.h"


template <class T>
concept PxClassifier = requires (T a, MaskByte b){
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
};

struct IsTransitionPx { 
  MaskByte comp_value;

  constexpr bool operator(MaskByte px){
    return px == comp_value;
  }
};

