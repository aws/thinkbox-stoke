# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required( VERSION 3.20 FATAL_ERROR )

project( Stoke )

find_package( thinkboxcmlibrary REQUIRED )
include( ThinkboxCMLibrary )
include( PrecompiledHeader )

add_library( stoke STATIC )

target_include_directories( stoke PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
target_include_directories( stoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )

file( GLOB_RECURSE H_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "include/*.h"
    "include/*.hpp"
)

file( GLOB_RECURSE CXX_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "src/*.cpp"
)

target_sources( stoke PRIVATE
    stdafx.cpp
    stdafx.h
    ${H_FILES}
    ${CXX_FILES}
)

# The Conan version of Boost was built with this, and it changes the library names.
# As a result, we need to set this to tell Boost to look for the right libraries to
# link against.
target_compile_definitions( stoke PUBLIC BOOST_AUTO_LINK_SYSTEM )

find_package( thinkboxlibrary REQUIRED )
find_package( magma REQUIRED )
find_package( Boost REQUIRED )
find_package( OpenEXR REQUIRED )
find_package( TBB REQUIRED )
find_package( tinyxml2 REQUIRED )
find_package( ZLIB REQUIRED )
find_package( OpenVDB REQUIRED )
find_package( HDF5 REQUIRED )
find_package( MKL REQUIRED )

target_include_directories( stoke PUBLIC ${thinkboxlibrary_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${magma_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${Boost_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${OpenEXR_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${TBB_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${tinyxml2_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${ZLIB_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${OpenVDB_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${HDF5_INCLUDE_DIRS} )
target_include_directories( stoke PUBLIC ${MKL_INCLUDE} )

target_link_libraries( stoke INTERFACE thinkboxlibrary::thinkboxlibrary )
target_link_libraries( stoke INTERFACE magma::magma )
target_link_libraries( stoke INTERFACE Boost::Boost )
target_link_libraries( stoke INTERFACE OpenEXR::OpenEXR )
target_link_libraries( stoke INTERFACE TBB::tbb )
target_link_libraries( stoke INTERFACE tinyxml2::tinyxml2 )
target_link_libraries( stoke INTERFACE ZLIB::ZLIB )
target_link_libraries( stoke INTERFACE OpenVDB::OpenVDB )
target_link_libraries( stoke INTERFACE HDF5::HDF5 )
target_link_libraries( stoke INTERFACE MKL::MKL )

target_compile_definitions( stoke PUBLIC OPENVDB_STATICLIB )

frantic_default_source_groups( stoke HEADERDIR include SOURCEDIR src )
frantic_common_platform_setup( stoke )

add_precompiled_header( stoke stdafx.h SOURCE_CXX stdafx.cpp )

# Disable optimization for the RelWithDebInfo configuration on Windows.
# This allows breakpoints to be hit reliably when debugging in Visual Studio.
if( WIN32 )
	target_compile_options( stoke PRIVATE "$<$<CONFIG:Release>:/O2>$<$<CONFIG:RelWithDebInfo>:/Od>" )
endif()

install( DIRECTORY include
		 DESTINATION "include/.."
		 FILES_MATCHING PATTERN "*.hpp"
)
install( TARGETS stoke
		 RUNTIME DESTINATION bin
		 LIBRARY DESTINATION lib
		 ARCHIVE DESTINATION lib
)
