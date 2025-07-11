#include "shared_utils.hh"

// FLTK Gui includes
#include <FL/fl_draw.H>
#include <FL/gl.h> // FLTK takes care of platform-specific GL stuff
// except GLU
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif


// Periodic space utility functions

// In periodic space, find the equivalent coordinates for Pose b (so b could be translated by 2 * r_upper) closest to Pose a
// r_upper describes 1/2 the side length of periodic space
Pose nearest_periodic(Pose a, Pose b, meters_t r_upper) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;

    bool dx_close = fabs(dx) <= r_upper;
    bool dy_close = fabs(dy) <= r_upper;

    Pose result = Pose(b.x, b.y, 0, 0);

    if (dx_close && dy_close) {
        return result;
    }

    if (!dx_close) {
        result.x = (dx >= 0 ? b.x - 2 * r_upper : b.x + 2 * r_upper);
    }

    if (!dy_close) {
        result.y = (dy >= 0 ? b.y - 2 * r_upper : b.y + 2 * r_upper);
    }

    return result;

}

// slower implementation of nearest_periodic function, kept for testing purposes
Pose nearest_periodic_slow(Pose cur_pos, Pose goal_pos, meters_t r_upper) {
    double s = 2 * r_upper;
    double xs [9] = {-s, -s, -s, 0, 0, 0, s, s, s};
    double ys [9] = {-s, 0, s, -s, 0, s, -s, 0, s};
    int closest_pos = 0;
    double closest_dist = std::numeric_limits<double>::infinity();
    for ( int i=0; i<9; i++ ) {
        Pose test_pos = Pose(goal_pos.x + xs[i], goal_pos.y + ys[i], 0, 0);
        double dist = cur_pos.Distance(test_pos);
        if (dist < closest_dist) {
        closest_dist = dist;
        closest_pos = i;
        }
    }
    return Pose(goal_pos.x + xs[closest_pos], goal_pos.y + ys[closest_pos], 0, 0);
}




