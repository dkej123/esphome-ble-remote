#include <map>
#include <string>
#include <utility>


struct UsagePage {
  UsagePage(const char* name,
            std::map<uint16_t, const char *> usages)
      : name_(name), usages_(std::move(usages)){};
  const char* name_;
  std::map<uint16_t, const char *> usages_;
};

// Built lazily on first use (function-local static) instead of as a global.
// Constructing this nested std::map during C++ static initialization
// (__libc_init_array, before app_main) was aborting on ESP-IDF 5.x /
// ESPHome 2026.5: the recursive std::map copies threw (bad_alloc) before the
// app could even print its banner, giving an endless boot/abort loop.
const std::map<uint8_t, UsagePage>& get_usage_pages();
