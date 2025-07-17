#include "astar_utils.hh"

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

void SpaceDiscretizer::initialize_space() {
    // generate cells
    float cur_x = -1.0 * space_r;
    for (int idx = 0; idx < cells_per_side; idx++) {
        float cur_y = -1.0 * space_r;
        for (int idy = 0; idy < cells_per_side; idy++) {
            cells[idx][idy] = new SpaceUnit(cur_x, cur_y, cell_width);
            cells[idx][idy]->id = SiteID(idx, idy);
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

    // populate each cell's neighbors and me member
    for (int idx = 0; idx < cells_per_side; idx++) {
        for (int idy = 0; idy < cells_per_side; idy++) {
            cells[idx][idy]->neighbors_and_me = cells[idx][idy]->neighbors;
            cells[idx][idy]->neighbors_and_me.push_back(cells[idx][idy]);
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



// get coordinate position of a SiteID
Pose SpaceDiscretizer::get_pos_as_pose(SiteID site_id) {
    SpaceUnit *su = cells[site_id.idx][site_id.idy];
    return Pose(su->x, su->y, 0, 0);
}