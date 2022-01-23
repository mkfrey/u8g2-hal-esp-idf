cmake_minimum_required(VERSION 3.13.4)

# In-place use of library in this example
list(APPEND EXTRA_COMPONENT_DIRS " ${CMAKE_CURRENT_LIST_DIR}/..")

# Project
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(u8g2_hal)
