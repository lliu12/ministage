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
    bool diags_take_longer;
    int max_replan_depth; // not done
    meters_t r_upper; /// the length of the arena is 2 * r_upper, and the x and y coordinates range from -r_upper to +r_upper

    // // neighbor search settings
    // meters_t cells_range; // coordinate range covered by cells with side length < sensing range
    int cells_per_side; // split the (r_upper)^2 square region into (sites_per_side)^2 cells for tracking agents in
    float cell_width; // distance between neighboring sites

    float dt; // how much to update by during each step
    int time_steps;
    bool verbose;

    // for agents
    meters_t sensing_range;
    radians_t sensing_angle;
    meters_t goal_tolerance;

    // for gui
    float gui_speedup;
    int gui_zoom;
    bool gui_draw_cells, gui_draw_footprints, gui_random_colors;

    // for saving data
    float save_data_interval; // leave as empty string to not save data
    std::string outfile_name;
    std::string addtl_data; // optional label or additional data to save with this simulation

} sim_params;


// container for the x,y indices of a site in the SpaceDiscretizer array
class SiteID {
    public:
    int idx, idy;

    SiteID(int x, int y) : idx(x), idy(y) { /*empty*/}

    SiteID() : idx(0), idy(0) { /*empty*/}

    ~SiteID() {}

    inline SiteID operator+(const SiteID &s) const { return SiteID(idx + s.idx, idy + s.idy); }

    inline SiteID operator-(const SiteID &s) const { return SiteID(idx - s.idx, idy - s.idy); }

    bool operator<(const SiteID &s) const { return ((idy * idy + idx * idx) < (s.idy * s.idy + s.idx * s.idx)); }

    bool operator==(const SiteID &s) const { return (idx == s.idx && idy == s.idy); }

    bool operator!=(const SiteID &s) const { return (idx != s.idx || idy != s.idy); }

    radians_t angle() const { return atan2(idy, idx); }

    // float l1_norm(const SiteID &s) const { return abs(idx - s.idx) + abs(idy - s.idy); }

    // float l2_norm(const SiteID &s) const { return hypot(idx - s.idx, idy - s.idy); }

    static SiteID random(int max_idx, int max_idy) { return SiteID(Random::get_unif_int(0, max_idx), Random::get_unif_int(0, max_idy)); }

    virtual void print(const char *prefix) const { printf("%s site id [x index:%i y index:%i]\n", prefix, idx, idy); }

    struct hash
    {
        size_t operator()(const SiteID &s) const
        {
            size_t xHash = std::hash<int>()(s.idx);
            size_t yHash = std::hash<int>()(s.idy) << 1;
            return xHash ^ yHash;
        }
    };

};

// Given a global difference between two cells that should be neighbors on the torus, 
// recover the direction we step in (e.g. -9, 1 a step across the boundary becomes 1,1)
inline SiteID recover_periodic_step(SiteID a, int cells_per_side) {
    int x = ((a.idx + cells_per_side) % cells_per_side);
    x = x > 1 ? x - cells_per_side : x;
    int y = ((a.idy + cells_per_side) % cells_per_side);
    y = y > 1 ? y - cells_per_side : y;

    return SiteID(x,y);
};

// Given an out of bounds position, wrap it in periodic space to be valid
inline SiteID wrap_periodic(SiteID a, int cells_per_side) {
    int wrap_idx = (a.idx + cells_per_side) % cells_per_side;
    int wrap_idy = (a.idy + cells_per_side) % cells_per_side;
    return SiteID(wrap_idx, wrap_idy);
}


// SpaceUnit class
class SpaceUnit {
    public:

    // Constructor with three parameters specifying unit position and width
    SpaceUnit(meters_t xmin, meters_t y_min, meters_t w);

    // Constructor with four parameters specifying bounds of site
    SpaceUnit(float x_min, float x_max, float y_min, float y_max);

    ~SpaceUnit(){}

    SiteID id;
    meters_t x, y, width; // x-coord of middle of cell, y-coord of middle of cell, width of cell
    meters_t xmin, xmax, ymin, ymax; // cell bounds
    std::vector<SpaceUnit *> neighbors; // pointers to neighboring cells
    std::vector<SpaceUnit *> neighbors_and_me;
    bool is_outer; // unit is on the outer boundary of the square arena and will need to be wrapped across to its neighbors for torus

    // add neighbor
    void add_neighbor(SpaceUnit* nbr) {
        neighbors.push_back(nbr);
    }

    // draw on canvas
    void draw();

    // TODO: draw connections to neighbors
    // void draw_connections();
};



// SpaceDiscretizer class
// Assumes the center of space is the origin
class SpaceDiscretizer {
    public:

    int cells_per_side;
    meters_t space_r; // radius aka 1/2 the side length of the space being discretized
    meters_t cell_width;
    bool periodic;
    bool connect_diagonals; // whether to add diagonal neighbors. if false only add neighbors sharing sides.
    std::vector<std::vector<SpaceUnit *>> cells;

    Pose get_pos_as_pose(SiteID site_id);

    SpaceDiscretizer(meters_t r_upper, int u_per_side, bool periodic, bool diags);
    ~SpaceDiscretizer();

    

    private:
    virtual void initialize_space();
    // virtual void new_cell(meters_t xmin, meters_t ymin, meters_t width);
    void cell_neighbor_helper(int idx, int idy, int nbr_idx, int nbr_idy);
};






#endif