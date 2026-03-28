#pragma once

#include "unit/unit.h"

namespace geom::units {

struct RadianTag {};
using RadianRep = double;

using Radian = unit::Unit<RadianTag, RadianRep>;

}// namespace geom::units
