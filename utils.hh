#ifndef UTILS_H
#define UTILS_H


#include <cmath>
#include <iostream>
#include <vector>
#include "random.hh"


/** Metres: floating point unit of distance */
typedef double meters_t;

/** Radians: unit of angle */
typedef double radians_t;

// Agent and simulation parameters
typedef struct {
    int num_agents;

    bool periodic; ///< use periodic boundary conditions
    bool circle_arena; /// if false, arena is square shaped
    meters_t r_upper; /// outer radius for circle, sidelength / 2 for square
    meters_t r_lower; /// inner radius for goals generated on a 2D ring
    meters_t periodic_bounds; ///< x and y axis value where periodic bounds are enforced

    meters_t model_init_mindist; ///< distance to perturb models away from each other when using AdjustModelPositions function
    int model_init_iters; ///< maximum iterations when using AdjustModelPositions function
    uint64_t dt; // how much to update by during each step
    bool verbose;

    // for agents
    meters_t sensing_range;
    radians_t sensing_angle;

    meters_t stopdist;
    float cruisespeed;

    // for noisy walk
    float anglenoise;
    float anglebias;
    int runsteps;
    bool randomize_runsteps;
    int turnspeed; 

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

    // add motion using local angle
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
class SimulationData {
    public: 
        SimulationData();
        SimulationData(sim_params sim_params);
        ~SimulationData();

        sim_params sp;
        uint64_t sim_time;
        std::vector <Pose *> positions; // ordered by Agent id


        void reset();
        std::vector <sensor_result> sense(int agent_id, Pose agent_pos, meters_t sensing_range, radians_t sensing_angle);

};


#endif