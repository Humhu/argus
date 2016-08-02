#include "paraset/ParamChecks.hpp"

#include <limits>

namespace argus
{

template <>
long ProjectionTraits<long>::min_increment() { return 1; }

template <>
double ProjectionTraits<double>::min_increment() { return std::numeric_limits<double>::epsilon(); }

}