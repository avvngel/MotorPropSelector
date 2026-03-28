#pragma once

#include "px_classifier.h"

template <class T>
concept PxAgg = requires (
  T a, 
  MaskByte b, 
  cv::Point pt
){
  { a(b, pt) } -> std::same_as_v<bool>;
  { a.hit };
  requires PxClassifier<
    std::remove_cvref_t<decltype<a.hit>>
  >;
};

template <PxClassifier C>
struct NoAgg {
  C hit{};

  constexpr bool operator(MaskByte px, cv::Point pt){
    return hit(px);
  }
};

template <PxClassifier C>
struct All {
  bool all_hits{true};
  C hit{};
  
  constexpr bool operator(MaskByte px, cv::Point pt){
    all_hits &&= hit(px);
    return all_hits;
  }
};

template <class T>
concept ScanAgg = requires (T a, ScanRes r){
  { T(r) } -> std::same_as_v<ScanRes>;
};

struct CollectHits {
  
  std::vector<cv::Point> hits
    
  ScanRes operator(ScanRes res){
    if (res.found){ 
      hits.emplace_back(res.pt);
    }
    return res;
  }
}



