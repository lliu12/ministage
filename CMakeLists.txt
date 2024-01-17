cmake_minimum_required(VERSION 3.10)

# set project name
project(Run_MiniStage)

# require c++ 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# # add library
# add_library(MiniStage ministage.hh)

# add the executable
set(SOURCES 
        run_ministage.cc 
        simulation_data.cc 
        agents.cc
        simulation_manager.cc)
        
add_executable(Run_MiniStage ${SOURCES})

# # # TODO 3: Use target_link_libraries to link the library to our executable
# target_link_libraries(Run_MiniStage PUBLIC MiniStage)

# # add the binary tree to the search path for include files
# target_include_directories(Run_MiniStage PUBLIC
#     "${PROJECT_BINARY_DIR}"
#     # "${PROJECT_SOURCE_DIR}/MathFunctions"
#     )


# add_executable(test ${SOURCES})


