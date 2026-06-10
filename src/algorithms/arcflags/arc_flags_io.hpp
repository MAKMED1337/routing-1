#pragma once

#include "algorithms/arcflags/arc_flags.hpp"

#include <string>

namespace transport::arcflags {

[[nodiscard]] bool save_arc_flags(const ArcFlagsPreprocessedData &data, const std::string &path);
[[nodiscard]] ArcFlagsPreprocessedData load_arc_flags(const std::string &path);

} // namespace transport::arcflags
