#include <cassert>
#include <cstdint>
#include <memory>

#include "components/ble_client_hid/hid_parser.h"

using esphome::ble_client_hid::HIDReportMap;

namespace {

constexpr uint8_t REPORT_WITHOUT_ID[] = {
    0x05, 0x0C,  // Usage Page (Consumer)
    0x09, 0xE9,  // Usage (Volume Increment)
    0x15, 0x00,  // Logical Minimum (0)
    0x25, 0x01,  // Logical Maximum (1)
    0x75, 0x01,  // Report Size (1)
    0x95, 0x01,  // Report Count (1)
    0x81, 0x02,  // Input (Data, Variable, Absolute)
};

constexpr uint8_t REPORT_WITH_ID[] = {
    0x05, 0x0C, 0x09, 0xE9, 0x15, 0x00, 0x25, 0x01,
    0x85, 0x01, 0x75, 0x01, 0x95, 0x01, 0x81, 0x02,
};

void test_variable_press_release_and_duplicate() {
  std::unique_ptr<HIDReportMap> map(HIDReportMap::parse_report_map_data(
      REPORT_WITHOUT_ID, sizeof(REPORT_WITHOUT_ID)));
  assert(map != nullptr);

  const uint8_t pressed[] = {0x01};
  auto values = map->parse(pressed, sizeof(pressed));
  assert(values.size() == 1);
  assert(values[0].usage.page == 0x0C);
  assert(values[0].usage.usage == 0xE9);
  assert(values[0].value == 1);
  assert(map->parse(pressed, sizeof(pressed)).empty());

  const uint8_t released[] = {0x00};
  values = map->parse(released, sizeof(released));
  assert(values.size() == 1);
  assert(values[0].value == 0);
}

void test_short_and_unknown_reports_are_ignored() {
  std::unique_ptr<HIDReportMap> map(HIDReportMap::parse_report_map_data(
      REPORT_WITH_ID, sizeof(REPORT_WITH_ID)));
  assert(map != nullptr);

  const uint8_t short_report[] = {0x01};
  assert(map->parse(short_report, sizeof(short_report)).empty());

  const uint8_t unknown_report[] = {0x02, 0x01};
  assert(map->parse(unknown_report, sizeof(unknown_report)).empty());

  const uint8_t valid_report[] = {0x01, 0x01};
  assert(map->parse(valid_report, sizeof(valid_report)).size() == 1);
}

void test_truncated_report_map_is_safe() {
  const uint8_t truncated_map[] = {0x06, 0x0C};
  std::unique_ptr<HIDReportMap> map(HIDReportMap::parse_report_map_data(
      truncated_map, sizeof(truncated_map)));
  assert(map != nullptr);
  const uint8_t data[] = {0x01};
  assert(map->parse(data, sizeof(data)).empty());
}

}  // namespace

int main() {
  test_variable_press_release_and_duplicate();
  test_short_and_unknown_reports_are_ignored();
  test_truncated_report_map_is_safe();
  return 0;
}
