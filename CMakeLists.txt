cmake_minimum_required(VERSION 3.10)

# set project name
project(MiniStage)

# require c++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set optimization level
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# Add compiler flags to suppress deprecation warnings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
add_compile_options(-Wno-deprecated-declarations)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

# find FLTK
FIND_PACKAGE(FLTK REQUIRED)

# Check if FLTK is found
if(FLTK_FOUND)
    message(STATUS "FLTK found.") # Version: ${FLTK_VERSION}")
else()
    message(FATAL_ERROR "FLTK not found. Please make sure it is installed.")
endif()

# add the executable
add_executable(Run_MiniStage run_ministage.cc shared_utils.cc)
add_executable(MiniStage_Results get_ministage_results.cc shared_utils.cc)
add_executable(Run_Astar run_astar.cc shared_utils.cc)

# add source code library
add_subdirectory(src)
add_subdirectory(astar_src)

include_directories(${FLTK_INCLUDE_DIR} src astar_src)
target_link_libraries(Run_MiniStage PUBLIC src ${FLTK_LIBRARIES})
target_link_libraries(MiniStage_Results PUBLIC src ${FLTK_LIBRARIES})
target_link_libraries(Run_Astar PUBLIC astar_src ${FLTK_LIBRARIES})



# # add the binary tree to the search path for include files
# target_include_directories(Run_MiniStage PUBLIC
#     "${PROJECT_BINARY_DIR}"
#     # "${PROJECT_SOURCE_DIR}/subdirectory_name
#     )

