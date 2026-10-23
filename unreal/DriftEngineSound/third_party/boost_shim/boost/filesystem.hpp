// Minimal boost::filesystem shim over std::filesystem, just enough for
// piranha/src/path.cpp (engine-sim's scripting front end), so the build does
// not need Boost.
#pragma once
#include <filesystem>
#include <system_error>

namespace boost {
namespace filesystem {

class path : public std::filesystem::path {
public:
    path() = default;
    path(const std::filesystem::path &p) : std::filesystem::path(p) {}
    path(const std::string &s) : std::filesystem::path(s) {}
    path(const char *s) : std::filesystem::path(s) {}

    bool is_complete() const { return is_absolute(); }
    path parent_path() const { return path(std::filesystem::path::parent_path()); }
    path extension() const { return path(std::filesystem::path::extension()); }
    path stem() const { return path(std::filesystem::path::stem()); }
};

inline path operator/(const path &a, const path &b) {
    return path(static_cast<const std::filesystem::path &>(a) /
                static_cast<const std::filesystem::path &>(b));
}

inline bool exists(const path &p) {
    std::error_code ec;
    return std::filesystem::exists(p, ec);
}

// boost throws when neither path exists; never throw here.
inline bool equivalent(const path &a, const path &b) {
    std::error_code ec;
    const bool r = std::filesystem::equivalent(a, b, ec);
    if (ec) return a.lexically_normal() == b.lexically_normal();
    return r;
}

} // namespace filesystem
} // namespace boost
