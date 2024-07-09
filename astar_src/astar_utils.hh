#ifndef ASTAR_UTILS_H
#define ASTAR_UTILS_H

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <set>
#include "../random.hh"

// FLTK Gui includes
#include <FL/fl_draw.H>
#include <FL/gl.h> // FLTK takes care of platform-specific GL stuff
// except GLU
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif


/** Metres: floating point unit of distance */
typedef double meters_t;

/** Radians: unit of angle */
typedef double radians_t;

// Agent and simulation parameters
typedef struct {
    int num_agents;

    bool periodic; ///< use periodic boundary conditions
    meters_t r_upper; /// the length of the arena is 2 * r_upper, and the x and y coordinates range from -r_upper to +r_upper

    // // neighbor search settings
    // meters_t cells_range; // coordinate range covered by cells with side length < sensing range
    int sites_per_side; // split the (r_upper)^2 square region into (sites_per_side)^2 cells for tracking agents in
    float site_dist; // distance between neighboring sites

    float dt; // how much to update by during each step
    bool verbose;

    // for agents
    meters_t sensing_range;
    radians_t sensing_angle;
    meters_t goal_tolerance;
    float cruisespeed;

    // for gui
    float gui_speedup;
    int gui_zoom;
    bool gui_draw_cells, gui_draw_footprints;
    // bool gui_ind_colors; // todo

    // for saving data
    float save_data_interval; // leave as empty string to not save data
    std::string outfile_name;
    std::string addtl_data; // optional label or additional data to save with this simulation

} sim_params;


// Site class
// Represents a discrete site in space agents can be located at
class Site {
    public:
    Site(float x_pos, float y_pos, float site_width) {
        x = x_pos;
        y = y_pos;
        width = site_width;

        xmin = x - width / 2;
        xmax = x + width / 2;
        ymin = y - width / 2;
        ymax = y + width / 2;
    }

    ~Site(){}

    float x, y, width;
    float xmin, xmax, ymin, ymax; // site bounds used for drawing
    std::vector<Site *> neighbors; // neighboring sites
    bool is_outer; // site is on the outer boundary of the square arena and will need to be wrapped across to its neighbors for torus

    // add neighbor
    void add_neighbor(Site* nbr) {
        neighbors.push_back(nbr);
    }

    // draw on canvas
    void draw() {
        glBegin(GL_LINE_LOOP);               // Draw outline of cell, with no fill
        glColor4f(0.0f, 0.9, 0.0f, 0.2);    // different Green outline
        
        glVertex2f(xmin, ymin);              // x, y
        glVertex2f(xmax, ymin);
        glVertex2f(xmax, ymax);
        glVertex2f(xmin, ymax);
        glEnd();
    }

};



#endif