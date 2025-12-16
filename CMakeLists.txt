cmake_minimum_required(VERSION 3.10)

# set project name
project(MiniStage)

# set data directory
set(SIM_DATA_DIR "${CMAKE_SOURCE_DIR}/simulation_data")

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

# add source code library
add_subdirectory(src)
add_subdirectory(astar_src)

include_directories(${FLTK_INCLUDE_DIR} src astar_src)

file(GLOB SIMULATION_SCRIPTS
    "${CMAKE_SOURCE_DIR}/simulation_scripts/*.cc"
)


foreach(SRC_FILE ${SIMULATION_SCRIPTS})
    # Get the base name without extension
    get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)

    add_executable(${TARGET_NAME} ${SRC_FILE} shared_utils.cc)

    # Link correct library
    if(TARGET_NAME MATCHES "astar")
        target_link_libraries(${TARGET_NAME} PUBLIC astar_src ${FLTK_LIBRARIES})
    else()
        target_link_libraries(${TARGET_NAME} PUBLIC src ${FLTK_LIBRARIES})
    endif()

    target_compile_definitions(${TARGET_NAME} PRIVATE
        SIM_DATA_DIR=\"${SIM_DATA_DIR}\"
    )
endforeach()

