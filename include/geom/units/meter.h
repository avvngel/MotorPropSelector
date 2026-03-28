#pragma once

#include "unit/unit.h"

namespace geom::units {

struct MeterTag {};

using Meter = unit::Unit<MeterTag, double>;


}
