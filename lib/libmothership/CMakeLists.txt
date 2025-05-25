cmake_minimum_required(VERSION 3.16)
project(libmothership)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB HEADERS "include/*.h") 

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(mothership STATIC ${SOURCES})

# Test scripts
add_executable(test_dexch ${SOURCES} test/dexch.cpp)
add_executable(test_network ${SOURCES} test/network.cpp)
add_executable(test_quaternion ${SOURCES} test/test_quaternion.cpp)
add_executable(test_quaternion2 ${SOURCES} test/test_quaternion2.cpp)
add_executable(test_vector3 ${SOURCES} test/test_vector3.cpp)
add_executable(test_jacobian ${SOURCES} test/test_jacobian.cpp)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(test_network PRIVATE ws2_32)
endif()

