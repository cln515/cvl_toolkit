cmake_minimum_required (VERSION 3.10)

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++0x")

project(cvl_toolkit)

add_library(${PROJECT_NAME}  STATIC
	src/utility.cpp
	src/config.cpp
	src/ply_object.cpp
)

include_directories(
	include
)

find_package(nlohmann_json 3.2.0 REQUIRED)
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)

find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)


install(TARGETS ${PROJECT_NAME} 
	EXPORT cvl-toolkit_export
	LIBRARY ARCHIVE DESTINATION lib
	INCLUDES DESTINATION include 
)

install(FILES 
${CMAKE_CURRENT_SOURCE_DIR}/include/utility.h
${CMAKE_CURRENT_SOURCE_DIR}/include/config.h
${CMAKE_CURRENT_SOURCE_DIR}/include/ply_object.h
DESTINATION include)

install(EXPORT cvl-toolkit_export
	FILE cvl_toolkit-config.cmake
	DESTINATION cmake
	EXPORT_LINK_INTERFACE_LIBRARIES
)
