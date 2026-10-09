// core/math/src/tolerance.cpp
#include "CAD_0/math/tolerance.hpp"

#include <sstream>

namespace CAD_0::math {

std::string Tolerance::describe() const {
    std::ostringstream ss;
    switch (kind) {
        case ToleranceKind::Linear:  ss << "linear="  << value << " mm"; break;
        case ToleranceKind::Angular:  ss << "angular=" << value << " rad"; break;
        case ToleranceKind::Relative: ss << "rel="     << value; break;
    }
    return ss.str();
}

} // namespace CAD_0::math
