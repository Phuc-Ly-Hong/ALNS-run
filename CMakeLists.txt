cmake_minimum_required(VERSION 3.20)
project(DASTS2_VERSION9_C)
include(FetchContent)

if(POLICY CMP0135)
	cmake_policy(SET CMP0135 NEW)
	set(CMAKE_POLICY_DEFAULT_CMP0135 NEW)
endif()

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
message( STATUS "Fetching Thor..." )
FetchContent_MakeAvailable(json)

set(CMAKE_CXX_STANDARD 17)


add_executable(DASTS2_VERSION9_C src/Config.cpp src/Input.cpp src/Solution.cpp main.cpp src/Utils.cpp src/Utils.h src/Random.h src/tabuSearch/TaBuSearch.cpp src/multiLevel/MultiLevel.cpp)
target_link_libraries(DASTS2_VERSION9_C PRIVATE nlohmann_json::nlohmann_json)

