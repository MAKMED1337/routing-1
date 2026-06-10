#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"

#include <string>

namespace transport::ch {

[[nodiscard]] bool save_ch(const ContractionHierarchy &ch, const std::string &path);
[[nodiscard]] ContractionHierarchy load_ch(const std::string &path);

} // namespace transport::ch
