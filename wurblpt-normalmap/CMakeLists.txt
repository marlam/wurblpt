# Copyright (C) 2019, 2020, 2021, 2022
# Computer Graphics Group, University of Siegen (written by Martin Lambers)
# Copyright (C) 2022, 2023
# Martin Lambers <marlam@marlam.de>
#
# Copying and distribution of this file, with or without modification, are
# permitted in any medium without royalty provided the copyright notice and this
# notice are preserved. This file is offered as-is, without any warranty.

cmake_minimum_required(VERSION 3.12)

project(wurblpt-normalmap LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# works with gcc and clang:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -Ofast -DNDEBUG")

find_package(libwurblpt REQUIRED)
find_package(libtgd REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${TGD_INCLUDE_DIRS} ${WURBLPT_INCLUDE_DIRS})
link_directories(${TGD_LIBRARY_DIRS} ${WURBLPT_LIBRARY_DIRS})
add_executable(wurblpt-normalmap wurblpt-normalmap.cpp)
target_link_libraries(wurblpt-normalmap ${WURBLPT_LIBRARIES} ${TGD_LIBRARIES} OpenMP::OpenMP_CXX)
install(TARGETS wurblpt-normalmap RUNTIME DESTINATION bin)
