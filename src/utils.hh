#ifndef UTILS_H
#define UTILS_H

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


class Agent;



// Agent and simulation parameters
typedef struct {
    int num_agents;

    bool periodic; ///< use periodic boundary conditions
    bool circle_arena; /// if false, arena is square shaped
    meters_t r_upper; /// outer radius for circle, sidelength / 2 for square where goals are generated
    meters_t r_lower; /// inner radius for goals generated on a 2D ring
    // meters_t periodic_bounds; ///< x and y axis value where periodic bounds are enforced = r_upper

    // neighbor search settings
    meters_t cells_range; // coordinate range covered by cells with side length < sensing range
    int cells_per_side; // split the (r_upper)^2 square region into (cells_per_side)^2 cells for tracking agents in
    bool use_sorted_agents, use_cell_lists;
    meters_t cell_width;


    // meters_t model_init_mindist; ///< distance to perturb models away from each other when using AdjustModelPositions function
    // int model_init_iters; ///< maximum iterations when using AdjustModelPositions function
    float dt; // how much to update by during each step
    bool verbose;

    // for agents
    meters_t sensing_range;
    radians_t sensing_angle;
    meters_t goal_tolerance;

    float cruisespeed;

    // for noisy walk
    float anglenoise;
    float anglebias;
    int avg_runsteps;
    bool randomize_runsteps;
    float turnspeed; 

    // for only applying noise sometimes
    float noise_prob; // At each step where we draw a new direction, apply anglenoise with this probability
    bool conditional_noise; // If true, only apply noise (with noise_prob) if the robot is currently blocked


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



// utility functions for drawing & converting coordinates
inline void coord_shift(double x, double y, double z, double a) {
    glTranslatef(x, y, z);
    glRotatef(rtod(a), 0, 0, 1);
}

inline void pose_shift(const Pose &pose) {
    coord_shift(pose.x, pose.y, pose.z, pose.a);
}

inline void pose_inverse_shift(const Pose &pose) {
    coord_shift(0, 0, 0, -pose.a);
    coord_shift(-pose.x, -pose.y, -pose.z, 0);
}



// Cell class
// Used in cell lists for identifying neighbors
class Cell {
    public:
    Cell(float x_min, float x_max, float y_min, float y_max) {
        xmin = x_min;
        xmax = x_max;
        ymin = y_min;
        ymax = y_max;
    }

    ~Cell(){}

    // int row, col; // index cell's position in overall cell vector (although... does the cell need to know this?)
    float xmin, xmax, ymin, ymax; // bounds of the space enclosed by cell
    bool is_overflow_cell; // cell outside the given arena bounds
    bool is_outer_cell; // cell adjacent to the outside of the arena bounds
    std::vector<Agent *> occupants; // agents in this cell
    std::vector<Cell *> neighbors; // neighboring cells


    // check if a pose is in cell
    // a cell includes its bottom/left sides and excludes its top/right sides
    bool in_cell(Pose p) 
    {
        bool x_in_cell = (xmin <= p.x && p.x < xmax);
        bool y_in_cell = (ymin <= p.y && p.y < ymax);
        return x_in_cell && y_in_cell;
    }

    // reset cell between trials
    void reset() {
        occupants.clear();
    }

    // add neighbor
    void add_neighbor(Cell* nbr) {
        neighbors.push_back(nbr);
    }

    // draw on canvas
    void draw() {
        glBegin(GL_LINE_LOOP);               // Draw outline of cell, with no fill
        // if (is_outer_cell) {
        if (false) {
            glColor4f(0.1, 0.7, 0.2, 0.2);    // Green outline
        }
        else {
            glColor4f(0.0f, 0.9, 0.0f, 0.2);    // different Green outline
        }
        
        glVertex2f(xmin, ymin);              // x, y
        glVertex2f(xmax, ymin);
        glVertex2f(xmax, ymax);
        glVertex2f(xmin, ymax);
        glEnd();
    }

};




typedef struct {
    int id; // id of sensed neighbor
    meters_t dist_away;
} sensor_result;

typedef struct {
    bool in_cone;
    meters_t dist_away;
} cone_result;

// need to be strictly within the sensing range
inline cone_result in_vision_cone(Pose agent_pos, Pose nbr_pos, meters_t my_sensor_range, radians_t my_sensor_angle) {
    cone_result result;
    
    double dx = nbr_pos.x - agent_pos.x;
    double dy = nbr_pos.y - agent_pos.y;
    radians_t gamma = atan2(dy, dx); // angle between nbr_pos - agent_pos and x-axis
    radians_t angle_away_from_centerline = fabs(gamma - agent_pos.a);

    result.dist_away = hypot(dy, dx); 
    result.in_cone = (result.dist_away < my_sensor_range) && (angle_away_from_centerline < my_sensor_angle / 2.0);

    return result;
}



/// Simulation Data Class
// Stores data about simulation time and agent positions
// Computes sensing information
class SimulationData {
    public: 
        SimulationData(sim_params *sim_params);
        ~SimulationData();

        sim_params *sp;
        double sim_time;

        /** maintain a vector of agents sorted by pose.x, for quickly finding neighbors */
        std::vector<Agent *> agents_byx_vec;

        /** maintain a vector of agents sorted by pose.y, for quickly finding neighbors */
        std::vector<Agent *> agents_byy_vec;

        // 2D vector of cell pointers
        std::vector<std::vector<Cell *>> cells;

        // 1D vector of agent pointers
        std::vector <Agent *> agents;

        // Overflow cell for positions outside the range of cells in the vector
        Cell *overflow_cell;

        void update();

        void reset();

        // Find nearby agents to a given position
        std::vector<Agent *> find_nearby_sorted_agents(Pose *agent_pos);

        // Find nearby agents to a given position
        std::vector<Agent *> find_nearby_cell_lists(Pose *agent_pos);

        // Return what this agent would sense
        std::vector <sensor_result> sense(int agent_id, Pose agent_pos);

        struct ltx {
            bool operator()(const Agent *a, const Agent *b) const;
        };

        struct lty {
            bool operator()(const Agent *a, const Agent *b) const;
        };

        // Find which cell a position belongs to
        Cell* get_cell_for_pos(Pose *p);

        // Create cells and neighbor relations
        void init_cell_lists();

        // Add agents to correct cell lists
        void populate_cell_lists();

        // check that the two neighbor-finding implementations agree
        bool neighbor_functions_agree(int agent_id, Pose agent_pos);

        // check if the byx, byy vecs are properly sorted
        bool vecs_sorted();

    private:
        void cell_neighbor_helper(int idx, int idy, int nbr_idx, int nbr_idy);

};



#endif