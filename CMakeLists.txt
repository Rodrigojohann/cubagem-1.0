cmake_minimum_required(VERSION 3.5)

project(cubagem-gui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(PCL 1.7.1 REQUIRED)
find_package(CURL REQUIRED)

include_directories(VisionaryCommon VisionaryCommon/include)
add_subdirectory(VisionaryCommon)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CURL_INCLUDE_DIR})

set(project_SOURCES dlib/dlib/all/source.cpp main.cpp executer.cpp processor.cpp sensor.cpp connector.cpp regressor.cpp)

add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${CURL_LIBRARIES} sick_visionary_common ${PCL_LIBRARIES} Qt5::Widgets)
