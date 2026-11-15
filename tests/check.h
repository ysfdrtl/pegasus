#pragma once
// Minimal test helpers: every CHECK prints one line, main() returns the failure count.
#include <cstdio>
#include <string>

namespace pegasus_test {
inline int& failures() {
    static int f = 0;
    return f;
}
inline void check(bool ok, const std::string& what) {
    std::printf("  %-72s %s\n", what.c_str(), ok ? "OK" : "FAIL");
    if (!ok) failures()++;
}
inline void section(const char* name) { std::printf("%s\n", name); }
inline int finish(const char* suite) {
    std::printf("[%s] %s (%d hata)\n", suite, failures() ? "BASARISIZ" : "TUM TESTLER GECTI", failures());
    return failures() ? 1 : 0;
}
} // namespace pegasus_test

#define CHECK(cond, ...) ::pegasus_test::check(bool(cond), __VA_ARGS__)
using pegasus_test::section;
