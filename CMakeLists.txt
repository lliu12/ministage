cmake_minimum_required(VERSION 3.10)

# set project name
project(Run_MiniStage)

# require c++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add compiler flags to suppress deprecation warnings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
add_compile_options(-Wno-deprecated-declarations)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

# add the executable
set(SOURCES 
        run_ministage.cc 
        simulation_data.cc 
        agents.cc
        simulation_manager.cc
        canvas.cc)
    
add_executable(Run_MiniStage ${SOURCES})

# find FLTK
# include(findFLTK)
FIND_PACKAGE(FLTK REQUIRED)

# Check if FLTK is found
if(FLTK_FOUND)
    message(STATUS "FLTK found.") # Version: ${FLTK_VERSION}")
    include_directories(${FLTK_INCLUDE_DIR})
    target_link_libraries(Run_MiniStage ${FLTK_LIBRARIES})
else()
    message(FATAL_ERROR "FLTK not found. Please make sure it is installed.")
endif()



# # # TODO 3: Use target_link_libraries to link the library to our executable
# target_link_libraries(Run_MiniStage PUBLIC MiniStage)

# # add the binary tree to the search path for include files
# target_include_directories(Run_MiniStage PUBLIC
#     "${PROJECT_BINARY_DIR}"
#     # "${PROJECT_SOURCE_DIR}/subdirectory_name
#     )


# add_executable(test ${SOURCES})


