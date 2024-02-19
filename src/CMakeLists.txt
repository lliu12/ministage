# Add all source files from the src directory to the library
# file(GLOB LIB_SOURCES "src/*.cc")

set(SOURCES 
        simulation_data.cc 
        agents.cc
        simulation_manager.cc
        canvas.cc)

# find FLTK
FIND_PACKAGE(FLTK REQUIRED)

# Check if FLTK is found
if(FLTK_FOUND)
    message(STATUS "FLTK found.") # Version: ${FLTK_VERSION}")
    include_directories(${FLTK_INCLUDE_DIR})
else()
    message(FATAL_ERROR "FLTK not found. Please make sure it is installed.")
endif()

# Create the library target
add_library(src ${SOURCES})

