#pragma once

#include "rep_cast_policy" 
#include "detail/rep_caster.h"

namespace unit {


template <
  class Tag,
  class RepIn,
  class RepOut,
  RepCastPolicy P = RepCastPolicy::kStaticCast
>
constexpr Unit<Tag, RepOut> rep_cast(
  const Unit<Tag, RepIn> in
){
  return Unit<Tag, RepOut>(
    detail::RepCaster<P>::cast<RepOut>(in.val)
  );
}

   
} // namespace utils
