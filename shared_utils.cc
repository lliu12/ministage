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


// SpaceUnit function definitions


// Constructor with three parameters specifying unit position and width
SpaceUnit::SpaceUnit(meters_t x_min, meters_t y_min, meters_t w) {
    xmin = x_min;
    ymin = y_min;
    width = w;

    x = xmin + width / 2;
    xmax = xmin + width;
    y = ymin + width / 2;
    ymax = ymin + width;
}

// Constructor with four parameters specifying bounds of site
SpaceUnit::SpaceUnit(float x_min, float x_max, float y_min, float y_max) {
    xmin = x_min;
    xmax = x_max;
    ymin = y_min;
    ymax = y_max;

    x = (x_min + x_max) / 2.0;
    y = (y_min + y_max) / 2.0;
    width = fabs(x_max - x_min);

    if (fabs(x_max - x_min) != fabs(y_max - y_min)) {
        printf("Error: SpaceUnit has different x and y widths.");
    }
}

// draw on canvas
void SpaceUnit::draw() {
    glBegin(GL_LINE_LOOP);               // Draw outline of cell, with no fill
    glColor4f(0.0f, 0.9, 0.0f, 0.2);    // different Green outline
    
    glVertex2f(xmin, ymin);              // x, y
    glVertex2f(xmax, ymin);
    glVertex2f(xmax, ymax);
    glVertex2f(xmin, ymax);
    glEnd();
}



// SpaceDiscretizer function definitions


SpaceDiscretizer::SpaceDiscretizer(meters_t r_upper, int u_per_side, bool is_periodic, bool diags) 
    : cells(u_per_side, std::vector<SpaceUnit *>(u_per_side))
{
    space_r = r_upper;
    cells_per_side = u_per_side;
    cell_width = 2 * space_r / cells_per_side;
    connect_diagonals = diags;
    periodic = is_periodic;

    initialize_space();
}

SpaceDiscretizer::~SpaceDiscretizer() {
    for (const auto& row : cells) {
        for (SpaceUnit *c : row) {
            delete c;
        }
    }
}

// void SpaceDiscretizer::new_cell(meters_t xmin, meters_t ymin, meters_t width) {

// }

void SpaceDiscretizer::initialize_space() {
    // generate cells
    float cur_x = -1.0 * space_r;
    for (int idx = 0; idx < cells_per_side; idx++) {
        float cur_y = -1.0 * space_r;
        for (int idy = 0; idy < cells_per_side; idy++) {
            cells[idx][idy] = new SpaceUnit(cur_x, cur_y, cell_width);
            cells[idx][idy]->is_outer = (idx == 0 || idy == 0 || 
                                            idx == cells_per_side - 1 || idy == cells_per_side - 1);
            cur_y += cell_width;
        }
        cur_x += cell_width;
    }

    // populate each cell's vector of neighbors
    // each cell initializes the linking for its neighbor to the top, right, and upper diagonals (if that cell exists)
    for (int idx = 0; idx < cells_per_side; idx++) {
        for (int idy = 0; idy < cells_per_side; idy++) {
            // link to neighbor on right
            cell_neighbor_helper(idx, idy, idx + 1, idy);

            // link to neighbor above
            cell_neighbor_helper(idx, idy, idx, idy + 1);

            if (connect_diagonals) {
                // link to upper right diagonal neighbor
                cell_neighbor_helper(idx, idy, idx + 1, idy + 1);

                // link to upper left diagonal neighbor
                cell_neighbor_helper(idx, idy, idx - 1, idy + 1);
            }

            // // link to overflow cell
            // if (cells[idx][idy]->is_outer_cell) {
            //     // link to overflow cell
            //     cells[idx][idy]->add_neighbor(overflow_cell);
            //     overflow_cell->add_neighbor(cells[idx][idy]);
            // }
        }
    }
}


// add the two cells as neighbors (two-way) if they are a valid pair (handling periodic cell wrapping)
void SpaceDiscretizer::cell_neighbor_helper(int idx, int idy, int nbr_idx, int nbr_idy) {
    int cps = cells_per_side;
    // if neighbor cell is beyond cell array bounds, only add them (after wrapping) if simulation is periodic
    if (nbr_idx >= cps || nbr_idx < 0 || nbr_idy >= cps || nbr_idy < 0) {
        if (periodic) {
                SpaceUnit *wrapped_nbr = cells[(nbr_idx + cps) % cps][(nbr_idy + cps) % cps];
                cells[idx][idy]->add_neighbor(wrapped_nbr);
                wrapped_nbr->add_neighbor(cells[idx][idy]);
        }
    }

    // otherwise make the cells neighbors as normal
    else {
        cells[idx][idy]->add_neighbor(cells[nbr_idx][nbr_idy]);
        cells[nbr_idx][nbr_idy]->add_neighbor(cells[idx][idy]);
    }

}

