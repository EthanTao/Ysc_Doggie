cmake_minimum_required(VERSION 3.10)
project(lite3_recovery_offline LANGUAGES CXX)

find_package(Threads REQUIRED)
add_library(recovery INTERFACE)
target_include_directories(recovery INTERFACE include)
target_compile_features(recovery INTERFACE cxx_std_17)
target_link_libraries(recovery INTERFACE Threads::Threads)

enable_testing()
add_executable(recovery_tests tests/recovery_tests.cpp)
target_link_libraries(recovery_tests PRIVATE recovery)
target_compile_options(recovery_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME recovery_stop_and_trajectory COMMAND recovery_tests)

add_executable(recovery_replay tests/replay.cpp)
target_link_libraries(recovery_replay PRIVATE recovery)
target_compile_options(recovery_replay PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(transport_mock_tests tests/transport_mock_tests.cpp)
target_link_libraries(transport_mock_tests PRIVATE recovery)
target_compile_options(transport_mock_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME recovery_transport_mock COMMAND transport_mock_tests)
