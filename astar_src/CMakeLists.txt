# Add all source files from the src directory to the library
# file(GLOB LIB_SOURCES "src/*.cc")

set(SOURCES 
    astar_manager.cc
	astar_canvas.cc
	astar_agent.cc
	astar_planner.cc
    astar_utils.cc
	../shared_utils.cc)

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
add_library(astar_src ${SOURCES})

