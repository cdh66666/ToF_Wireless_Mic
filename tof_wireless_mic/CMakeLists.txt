cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options(-Wno-ignored-qualifiers)
idf_build_set_property(MINIMAL_BUILD ON)

project(tof_wireless_mic)