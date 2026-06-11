#pragma once

#include "algorithms/arcflags/arc_flags.hpp"

#include <filesystem>

namespace transport::arcflags {

[[nodiscard]] bool save_arc_flags(const ArcFlagsPreprocessedData &data, const std::filesystem::path &path);
[[nodiscard]] ArcFlagsPreprocessedData load_arc_flags(const std::filesystem::path &path);

} // namespace transport::arcflags
