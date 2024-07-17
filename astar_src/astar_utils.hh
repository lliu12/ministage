#ifndef ASTAR_UTILS_H
#define ASTAR_UTILS_H

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <set>
#include "../random.hh"
#include "../shared_utils.hh"

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
    bool diags;
    meters_t r_upper; /// the length of the arena is 2 * r_upper, and the x and y coordinates range from -r_upper to +r_upper

    // // neighbor search settings
    // meters_t cells_range; // coordinate range covered by cells with side length < sensing range
    int cells_per_side; // split the (r_upper)^2 square region into (sites_per_side)^2 cells for tracking agents in
    float cell_width; // distance between neighboring sites

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
    bool gui_draw_cells, gui_draw_footprints, gui_random_colors;
    // bool gui_ind_colors; // todo

    // for saving data
    float save_data_interval; // leave as empty string to not save data
    std::string outfile_name;
    std::string addtl_data; // optional label or additional data to save with this simulation

} sim_params;


// // Site class
// // Represents a discrete site in space agents can be located at
// class Site {
//     public:
//     Site(float x_pos, float y_pos, float site_width) {
//         x = x_pos;
//         y = y_pos;
//         width = site_width;

//         xmin = x - width / 2;
//         xmax = x + width / 2;
//         ymin = y - width / 2;
//         ymax = y + width / 2;
//     }

//     ~Site(){}

//     float x, y, width;
//     float xmin, xmax, ymin, ymax; // site bounds used for drawing
//     std::vector<Site *> neighbors; // neighboring sites
//     bool is_outer; // site is on the outer boundary of the square arena and will need to be wrapped across to its neighbors for torus

//     // add neighbor
//     void add_neighbor(Site* nbr) {
//         neighbors.push_back(nbr);
//     }

//     // draw on canvas
//     void draw() {
//         glBegin(GL_LINE_LOOP);               // Draw outline of cell, with no fill
//         glColor4f(0.0f, 0.9, 0.0f, 0.2);    // different Green outline
        
//         glVertex2f(xmin, ymin);              // x, y
//         glVertex2f(xmax, ymin);
//         glVertex2f(xmax, ymax);
//         glVertex2f(xmin, ymax);
//         glEnd();
//     }

// };


// container for the x,y indices of a site in the SpaceDiscretizer array
class SiteID {
    public:
    int idx, idy;

    SiteID(int x, int y) : idx(x), idy(y) { /*empty*/}

    SiteID() : idx(0), idy(0) { /*empty*/}

    ~SiteID() {}

    inline SiteID operator+(const SiteID &s) const { return SiteID(idx + s.idx, idy + s.idy); }

    bool operator<(const SiteID &s) const { return ((idy * idy + idx * idx) < (s.idy * s.idy + s.idx * s.idx)); }

    bool operator==(const SiteID &s) const { return (idx == s.idx && idy == s.idy); }

    bool operator!=(const SiteID &s) const { return (idx != s.idx || idy != s.idy); }

    float l1_norm(const SiteID &s) const { return abs(idx - s.idx) + abs(idy - s.idy); }

    float l2_norm(const SiteID &s) const { return hypot(idx - s.idx, idy - s.idy); }

    static SiteID random(int max_idx, int max_idy) { return SiteID(Random::get_unif_int(0, max_idx), Random::get_unif_int(0, max_idy)); }

    virtual void print(const char *prefix) const { printf("%s site id [x index:%i y index:%i]\n", prefix, idx, idy); }

};



#endif