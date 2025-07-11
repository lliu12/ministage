#ifndef SHARED_UTILS_H
#define SHARED_UTILS_H

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <vector>
#include <set>
#include "random.hh"
#include <stdexcept>

// FLTK Gui includes
#include <FL/fl_draw.H>
#include <FL/gl.h> // FLTK takes care of platform-specific GL stuff
// except GLU
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif



// Units

/** Metres: floating point unit of distance */
typedef double meters_t;

/** Radians: unit of angle */
typedef double radians_t;


// Utility Functions
/** Normalize an angle to within +/_ M_PI. */
inline double normalize(double a)
{
    while (a <= -M_PI)
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

// In periodic space, find the equivalent coordinates for Pose b (so b could be translated by 2 * r_upper) closest to Pose a
// r_upper describes 1/2 the side length of periodic space
Pose nearest_periodic(Pose a, Pose b, meters_t r_upper);

// slower implementation of nearest_periodic function, kept for testing purposes
Pose nearest_periodic_slow(Pose cur_pos, Pose goal_pos, meters_t r_upper);


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
    radians_t angle_away_from_centerline = fabs(normalize(gamma - agent_pos.a));

    result.dist_away = hypot(dy, dx); 
    result.in_cone = (result.dist_away < my_sensor_range) && (angle_away_from_centerline < my_sensor_angle / 2.0);

    return result;
}


// Color class
class Color {
    public:
    double r, g, b, a;
    Color(double r, double g, double b, double a) : r(r), g(g), b(b), a(a) {}
    Color() : r(1.0), g(0.0), b(0.0), a(1.0) {}
    ~Color() {};

    static Color RandomColor()
    // { return Color(drand48(), drand48(), drand48(), 1.0); } // generates the same colors every run
    { return Color(Random::get_unif_double(0, 1), Random::get_unif_double(0, 1), Random::get_unif_double(0, 1), 1.0);  }

};

#endif