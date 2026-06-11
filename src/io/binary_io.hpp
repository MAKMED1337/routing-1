#pragma once

#include <array>
#include <bit>
#include <istream>
#include <ostream>
#include <span>
#include <stdexcept>
#include <type_traits>

namespace transport::io {

template <typename T> void write_one(std::ostream &out, const T &value) {
    static_assert(std::is_trivially_copyable_v<T>);
    const auto bytes = std::bit_cast<std::array<char, sizeof(T)>>(value);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

template <typename T> bool read_one(std::istream &in, T &value) {
    static_assert(std::is_trivially_copyable_v<T>);
    std::array<char, sizeof(T)> bytes{};
    in.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    value = std::bit_cast<T>(bytes);
    return static_cast<bool>(in);
}

template <typename T> void write_span(std::ostream &out, std::span<const T> values) {
    static_assert(std::is_trivially_copyable_v<T>);
    out.write(reinterpret_cast<const char *>(values.data()), static_cast<std::streamsize>(values.size_bytes()));
}

template <typename T> void read_span(std::istream &in, std::span<T> values, const char *error_message) {
    static_assert(std::is_trivially_copyable_v<T>);
    in.read(reinterpret_cast<char *>(values.data()), static_cast<std::streamsize>(values.size_bytes()));
    if (!in) {
        throw std::runtime_error(error_message);
    }
}

} // namespace transport::io
