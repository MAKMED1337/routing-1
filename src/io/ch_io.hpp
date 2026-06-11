#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"

#include <filesystem>

namespace transport::ch {

[[nodiscard]] bool save_ch(const ContractionHierarchy &ch, const std::filesystem::path &path);
[[nodiscard]] ContractionHierarchy load_ch(const std::filesystem::path &path);

} // namespace transport::ch
