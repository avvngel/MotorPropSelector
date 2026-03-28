#pragma once

namespace unit {

enum class RepCastPolicy {
  kStaticCast,
  kNearest,
  kFloor,
  kCeil,
  kTowardZero
};

} // unit
