# pragma once

#include "scan_res.h"

namespace scan {

template <class T>
concept TermPolicy = requires (T a, const ScanRes res){
  { a(res) } -> std::same_as_v<bool>;
}

struct TermOnHit {
  constexpr bool operator(const ScanRes res){
    return res.found;
  }
};

struct NoTerm {
  constexpr bool operator(const ScanRes res){
    return false;
  }
};


} // namespace scan
