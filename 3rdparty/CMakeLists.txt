cmake_minimum_required(VERSION 3.5)
project(nlohmann)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

include_directories(nlohmann)
add_library(nlohmann nlohmann/json.hpp)