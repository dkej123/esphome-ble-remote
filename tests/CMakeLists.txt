cmake_minimum_required(VERSION 3.16)
project(ble_client_hid_tests LANGUAGES CXX)

enable_testing()

add_executable(test_hid_parser
  test_hid_parser.cpp
  ../components/ble_client_hid/hid_parser.cpp
)
target_compile_features(test_hid_parser PRIVATE cxx_std_17)
target_include_directories(test_hid_parser PRIVATE .. stubs)
target_compile_options(test_hid_parser PRIVATE -Wall -Wextra -Werror)

add_test(NAME hid_parser COMMAND test_hid_parser)
