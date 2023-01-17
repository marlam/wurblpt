# Copyright (C) 2019, 2020, 2021, 2022
# Computer Graphics Group, University of Siegen
# Written by Martin Lambers <martin.lambers@uni-siegen.de>
# Copyright (C) 2022, 2023
# Martin Lambers <marlam@marlam.de>
#
# Copying and distribution of this file, with or without modification, are
# permitted in any medium without royalty provided the copyright notice and this
# notice are preserved. This file is offered as-is, without any warranty.

cmake_minimum_required(VERSION 3.12)
set(CMAKE_FIND_PACKAGE_SORT_ORDER NATURAL)
set(CMAKE_FIND_PACKAGE_SORT_DIRECTION DEC)

# Project
project(libwurblpt LANGUAGES CXX)
set(WURLBPT_VERSION 0.0)
set(WURLBPT_LIBVERSION 0.0.0)
set(WURLBPT_SOVERSION 0)

# Requirements
set(CMAKE_CXX_STANDARD 20)
find_package(libtgd REQUIRED)

# The WURLBPT library
include_directories(${TGD_INCLUDE_DIRS} ${WURLBPT_INCLUDE_DIRS})
link_directories(${TGD_LIBRARY_DIRS} ${WURLBPT_LIBRARY_DIRS})
set(LIBWURLBPT_PUBLIC_HEADERS
    gvm.hpp
    constants.hpp
    color.hpp
    tangentspace.hpp
    transformation.hpp
    animation.hpp
    animation_keyframes.hpp
    prng.hpp
    sampler.hpp
    texture.hpp
    texture_image.hpp
    texture_noise.hpp
    envmap.hpp
    fresnel.hpp
    geometryproc.hpp
    material.hpp
    material_lambertian.hpp
    material_mirror.hpp
    material_glass.hpp
    material_modphong.hpp
    material_ggx.hpp
    material_phase_function_isotropic.hpp
    material_rgl.hpp powitacq.h powitacq_rgb.h
    mpi.hpp
    light_diffuse.hpp
    light_spot.hpp
    light_tof.hpp
    ray.hpp
    mesh.hpp
    sphere.hpp
    medium.hpp
    hitable.hpp
    hitable_medium.hpp
    hitable_sphere.hpp
    hitable_triangle.hpp
    aabb.hpp
    bvh.hpp
    sensor.hpp
    sensor_rgb.hpp
    sensor_tof_amcw.hpp
    optics.hpp
    camera.hpp
    scene_component.hpp
    scene.hpp
    generator.hpp
    import.hpp
    tiny_obj_loader.h
    wurblpt.hpp
    postproc.hpp)
install(FILES ${LIBWURLBPT_PUBLIC_HEADERS} DESTINATION include/wurblpt)
add_library(libwurblpt SHARED ${LIBWURLBPT_PUBLIC_HEADERS}
    tiny_obj_loader.cpp
    powitacq.cpp powitacq.inl powitacq_rgb.inl)

set_target_properties(libwurblpt PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
set_target_properties(libwurblpt PROPERTIES OUTPUT_NAME wurblpt)
set_target_properties(libwurblpt PROPERTIES VERSION ${WURLBPT_LIBVERSION})
set_target_properties(libwurblpt PROPERTIES SOVERSION ${WURLBPT_SOVERSION})
install(TARGETS libwurblpt
    LIBRARY DESTINATION lib${LIB_SUFFIX}
    ARCHIVE DESTINATION lib${LIB_SUFFIX}
)
include(CMakePackageConfigHelpers)
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX})
configure_package_config_file(
    libwurblptConfig.cmake.in ${CMAKE_BINARY_DIR}/libwurblptConfig.cmake
    INSTALL_DESTINATION lib${LIB_SUFFIX}/cmake/libwurblpt-{WURLBPT_VERSION}
    PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/libwurblptConfigVersion.cmake
    VERSION ${WURLBPT_VERSION}
    COMPATIBILITY SameMajorVersion
)
install(FILES
    ${CMAKE_BINARY_DIR}/libwurblptConfig.cmake
    ${CMAKE_BINARY_DIR}/libwurblptConfigVersion.cmake
    DESTINATION ${LIB_INSTALL_DIR}/cmake/libwurblpt-${WURLBPT_VERSION}
)
