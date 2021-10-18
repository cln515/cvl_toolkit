cmake_minimum_required (VERSION 3.10)

project(cvl_toolkit)

add_library(${PROJECT_NAME}  STATIC
	src/utility.cpp
)

include_directories(
	include
)

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)


install(TARGETS ${PROJECT_NAME} 
	EXPORT cvl-toolkit_export
	LIBRARY ARCHIVE DESTINATION lib
	INCLUDES DESTINATION include 
)

install(FILES 
${CMAKE_CURRENT_SOURCE_DIR}/include/utility.h
DESTINATION include)

install(EXPORT cvl-toolkit_export
	FILE cvl_toolkit-config.cmake
	DESTINATION cmake
	EXPORT_LINK_INTERFACE_LIBRARIES
)
