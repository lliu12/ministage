#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <iostream>
#include <vector>
#include <set>
#include "random.hh"

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


/** Metres: floating point unit of distance */
typedef double meters_t;

/** Radians: unit of angle */
typedef double radians_t;

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


    meters_t model_init_mindist; ///< distance to perturb models away from each other when using AdjustModelPositions function
    int model_init_iters; ///< maximum iterations when using AdjustModelPositions function
    float dt; // how much to update by during each step
    bool verbose;

    // for agents
    meters_t sensing_range;
    radians_t sensing_angle;
    meters_t goal_tolerance;

    meters_t stopdist;
    float cruisespeed;

    // for noisy walk
    float anglenoise;
    float anglebias;
    int avg_runsteps;
    bool randomize_runsteps;
    float turnspeed; 

    // for gui
    float gui_speedup;
    int gui_zoom;
    // bool gui_ind_colors; // todo

} sim_params;


// Utility Functions
/** Normalize an angle to within +/_ M_PI. */
inline double normalize(double a)
{
    while (a < -M_PI)
        a += 2.0 * M_PI;
    while (a > M_PI)
        a -= 2.0 * M_PI;
    return a;
}

/** take binary sign of a, either -1, or 1 if >= 0 */
inline int sgn(int a)
{
    return (a < 0 ? -1 : 1);
}

/** take binary sign of a, either -1.0, or 1.0 if >= 0. */
inline double sgn(double a)
{
    return (a < 0 ? -1.0 : 1.0);
}

/** convert an angle in radians to degrees. */
inline double rtod(double r)
{
  return (r * 180.0 / M_PI);
}

/** convert an angle in degrees to radians. */
inline double dtor(double d)
{
  return (d * M_PI / 180.0);
}



/// Pose class describing a position in space
/** Specify a 3 axis position, in x, y and heading. */
class Pose {
    public:
    meters_t x, y, z; ///< location in 3 axes
    radians_t a; ///< rotation about the z axis.

    Pose(meters_t x, meters_t y, meters_t z, radians_t a) : x(x), y(y), z(z), a(a) { /*empty*/}

    Pose() : x(0.0), y(0.0), z(0.0), a(0.0) { /*empty*/}

    virtual ~Pose() {}

    /** return a random pose within the bounding rectangle, with z=0 and
    angle random */
    static Pose Random(meters_t xmin, meters_t xmax, meters_t ymin, meters_t ymax)
    {
        return Pose(xmin + Random::get_unif_double(0, 1) * (xmax - xmin), ymin + Random::get_unif_double(0, 1) * (ymax - ymin), 0,
                    normalize(Random::get_unif_double(0, 1) * (2.0 * M_PI)));
    }

    /** Print pose in human-readable format on stdout
    @param prefix Character string to prepend to pose output
    */
    virtual void Print(const char *prefix) const
    {
        printf("%s pose [x:%.3f y:%.3f z:%.3f a:%.3f]\n", prefix, x, y, z, a);
    }

    std::string String() const
    {
        char buf[256];
        snprintf(buf, 256, "[ %.3f %.3f %.3f %.3f ]", x, y, z, a);
        return std::string(buf);
    }

    /** Returns true iff all components of the velocity are zero. */
    bool IsZero() const { return (!(x || y || z || a)); }

    /** Set the pose to zero [0,0,0,0] */
    void Zero() { x = y = z = a = 0.0; }

    // Add motion using local angle.
    // Warning: does not directly add x and y coordinates! Instead, p.x describes how much the agent moves 
    // forward in the agent's local angle POV.
    inline Pose operator+(const Pose &p) const
    {
        const double cosa = cos(a);
        const double sina = sin(a);

        return Pose(x + p.x * cosa - p.y * sina, y + p.x * sina + p.y * cosa, z + p.z,
                    normalize(a + p.a));
    }

    /// a < b iff a is closer to the origin than b
    bool operator<(const Pose &p) const
    {
        // return( hypot( y, x )^2 < hypot( otHer.y, other.x )^2);
        return ((y * y + x * x) < (p.y * p.y + p.x * p.x));
    }

    bool operator==(const Pose &other) const
    {
        return (x == other.x && y == other.y && z == other.z && a == other.a);
    }

    bool operator!=(const Pose &other) const
    {
        return (x != other.x || y != other.y || z != other.z || a != other.a);
    }

    meters_t Distance(const Pose &other) const { return hypot(x - other.x, y - other.y); }
};



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
        if (is_outer_cell) {
            glColor4f(0.1, 0.7, 0.2, 0.2);    // Green outline
        }
        else {
            glColor4f(0.0f, 1.0f, 0.0f, 0.2);    // Green outline
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
        bool all_stopped; 

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

};

// Color object
class Color {
    public:
    double r, g, b, a;
    Color(double r, double g, double b, double a) : r(r), g(g), b(b), a(a) {}
    Color() : r(1.0), g(0.0), b(0.0), a(1.0) {}
    ~Color() {};

    static Color RandomColor()
    { return Color(drand48(), drand48(), drand48(), 1.0); }

};



#endif