cmake_minimum_required(VERSION 2.8.8)

# Project name
project(kitti_parser)

# Include our cmake files
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)


# Include libraries
#find_package(LCM REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(OpenCV 3 REQUIRED core plot videoio ximgproc)


# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


# List all files, but the main file
set(LIBRARY_FILES
    src/kitti_parser/Parser.cpp
    src/kitti_parser/util/Loader.cpp
)


# Create the core library (excludes the main file)
add_library(kitti_parser STATIC ${LIBRARY_FILES})

# Now specify what type of library, a cpp and public
set_target_properties(kitti_parser PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(kitti_parser PUBLIC src)

# Include our header files
include_directories(src ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Create the main executable, and link libraries
add_executable(main_text src/main_text.cpp)
# Link libs ${LCM_LIBRARY} ${Boost_LIBRARIES}
target_link_libraries(main_text kitti_parser ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

# Create the main executable, and link libraries
add_executable(main_image src/main_image.cpp)
# Link libs ${LCM_LIBRARY} ${Boost_LIBRARIES}
target_link_libraries(main_image kitti_parser ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
