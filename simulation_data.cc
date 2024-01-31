#include "utils.hh"
#include "agents.hh"

// Constructor
SimulationData::SimulationData(sim_params *sim_params) 
    : cells(sim_params->cells_per_side, std::vector<Cell *>(sim_params->cells_per_side))
{
    sp = sim_params;
    sim_time = 0;

    // initialize cell lists
    if (sp->use_cell_lists) {
        init_cell_lists();
    }
}

// Destructor
SimulationData::~SimulationData(){}

// Reset
void SimulationData::reset() {
    all_stopped = false;
    sim_time = 0;

    // ensure agent lists are sorted
    if (sp->use_sorted_agents) {
        std::sort(agents_byx_vec.begin(), agents_byx_vec.end(), ltx());
        std::sort(agents_byy_vec.begin(), agents_byy_vec.end(), lty());
    }

    // populate cell lists
    if (sp->use_cell_lists) {
        populate_cell_lists();
    }
}


// function objects for comparing model positions
bool SimulationData::ltx::operator()(const Agent *a, const Agent *b) const
{
  const meters_t ax(a->get_pos().x);
  const meters_t bx(b->get_pos().x);
  // break ties using the pointer value to give a unique ordering
  return (ax == bx ? a < b : ax < bx);
}

bool SimulationData::lty::operator()(const Agent *a, const Agent *b) const
{
  const meters_t ay(a->get_pos().y);
  const meters_t by(b->get_pos().y);
  // break ties using the pointer value ro give a unique ordering
  return (ay == by ? a < b : ay < by);
}

// update fields
void SimulationData::update() {
    // small optimization: if everyone was blocked last step, no need to re-sort
    bool all_blocked_helper = true;
    for (Agent *a : agents_byx_vec) 
    { 
        if (a->fwd_speed != 0) {
            all_blocked_helper = false;
            break;
        }
    }

    if (!all_blocked_helper || sim_time == 0) {
        // sort the position lists
        if (sp->use_sorted_agents) {
            std::sort(agents_byx_vec.begin(), agents_byx_vec.end(), ltx());
            std::sort(agents_byy_vec.begin(), agents_byy_vec.end(), lty());
        }

        // update cell occupancy
        // a performance update for later: since agents move slowly, 
        if (sp->use_cell_lists) {
            populate_cell_lists();
        }
    }
    else {
        // print out first timestep everyone is blocked, 
        // but note that the system could become unblocked again due to in-place rotations
        if (!all_stopped) {
            // printf("everyone is blocked at time %f\n", sim_time);
        }
        all_stopped = true;
    }

    sim_time += sp->dt;
}

void SimulationData::populate_cell_lists() {
    // clear all current cell occupants
    for (const auto& row : cells) {
        for (Cell *c : row) {
            c->occupants.clear();
        }
    }

    // iterate through agents and make them occupants of the correct cell
    for (Agent *a : agents) 
        {
            Cell *cur_cell = get_cell_for_pos(a->cur_pos);
            cur_cell->occupants.push_back(a);
    }
}


Cell* SimulationData::get_cell_for_pos(Pose *p) {
    meters_t cr = sp->cells_range;
    meters_t cw = sp->cell_width;

    // if out of cell range, return overflow
    if (p->x < -1.0 * cr || p->x >= cr || p->y < -1.0 * cr || p->y >= cr) {
        // printf("in overflow \n");
        return overflow_cell;
    }
    else {
        meters_t dist_from_left = p->x - (-1.0 * cr);
        int idx = floor(dist_from_left / cw);

        meters_t dist_from_bottom = p->y - (-1.0 * cr);
        int idy = floor(dist_from_bottom / cw);

        // printf("idx: %i, idy: %i \n", idx, idy);

        return cells[idx][idy];
    }
}



void SimulationData::init_cell_lists() {
    overflow_cell = new Cell(sp->cells_range, -1.0 * sp->cells_range, sp->cells_range, -1.0 * sp->cells_range);
    overflow_cell->is_outer_cell = false;
    overflow_cell->is_overflow_cell = true;

    float cur_x = -1.0 * sp->cells_range;

    // set bounds for each cell
    // cells[0][0] is in the bottom left
    // idx = a, idy = b is the 0-indexed cell which is ath from the left and bth from the bottom 
    for (int idx = 0; idx < sp->cells_per_side; idx++) {
        float cur_y = -1.0 * sp->cells_range;
        for (int idy = 0; idy < sp->cells_per_side; idy++) {
            cells[idx][idy] = new Cell(cur_x, cur_x + sp->cell_width, cur_y, cur_y + sp->cell_width);
            cells[idx][idy]->is_outer_cell = (idx == 0 || idy == 0 || 
                                            idx == sp->cells_per_side - 1 || idy == sp->cells_per_side - 1);
            cells[idx][idy]->is_overflow_cell = false;

            cur_y += sp->cell_width;
        }
        cur_x += sp->cell_width;
    }

    // populate each cell's vector of cell neighbors
    // each cell initializes the linking for its neighbor to the top, right, and top-right diagonal (if that cell exists)
    for (int idx = 0; idx < sp->cells_per_side; idx++) {
        for (int idy = 0; idy < sp->cells_per_side; idy++) {
            if (idx < sp->cells_per_side - 1) {
                // link to neighbor on right
                cells[idx][idy]->add_neighbor(cells[idx + 1][idy]);
                cells[idx + 1][idy]->add_neighbor(cells[idx][idy]);
            }

            if (idy < sp->cells_per_side - 1) {
                // link to neighbor above
                cells[idx][idy]->add_neighbor(cells[idx][idy + 1]);
                cells[idx][idy + 1]->add_neighbor(cells[idx][idy]);
            }

            if (idx < sp->cells_per_side - 1 && idy < sp->cells_per_side - 1) {
                // link to upper right diagonal neighbor
                cells[idx][idy]->add_neighbor(cells[idx + 1][idy + 1]);
                cells[idx + 1][idy + 1]->add_neighbor(cells[idx][idy]);
            }

            if (idx > 0 && idy < sp->cells_per_side - 1) {
                // link to upper left diagonal neighbor
                cells[idx][idy]->add_neighbor(cells[idx - 1][idy + 1]);
                cells[idx - 1][idy + 1]->add_neighbor(cells[idx][idy]);
            }

            if (cells[idx][idy]->is_outer_cell) {
                // link to overflow cell
                cells[idx][idy]->add_neighbor(overflow_cell);
                overflow_cell->add_neighbor(cells[idx][idy]);
            }

            // if (sp->periodic) {
            //     // TODO
            // }
        }
    }
}

// // Find nearby agents to a given position
// std::vector<Agent *> SimulationData::find_nearby(Pose *agent_pos) {
//     std::vector<Agent *> nearby;

//     if (sp->use_sorted_agents) {

//     }

//     // if (sp->use_cell_lists) {
        
//     // }
// }


// Return who an agent with id agent_id and Pose agent_pos would sense in its cone-shaped field of view
std::vector <sensor_result> SimulationData::sense(int agent_id, Pose agent_pos, meters_t sensing_range, radians_t sensing_angle) {
    std::vector <sensor_result> result;

    // std::cout << "\nTesting fiducial sorting in sensor function..." << std::endl;
    // vecs_sorted();

    // find nearby neighbors with vectors
    // first, find a smaller collection of nearby neighbors
    double rng = sp->sensing_range;
    Pose gp = agent_pos;
    Agent edge(-1, sp, this); // dummy model used to find bounds in the sets
    std::vector<Agent *>::iterator xmin, xmax, ymin, ymax;

    edge.set_pos(Pose(gp.x - rng, gp.y, 0, 0)); // LEFT
    xmin = std::lower_bound(agents_byx_vec.begin(), agents_byx_vec.end(), &edge, ltx());

    edge.set_pos(Pose(gp.x + rng, gp.y, 0, 0)); // RIGHT
    xmax = std::upper_bound(agents_byx_vec.begin(), agents_byx_vec.end(), &edge, ltx());

    edge.set_pos(Pose(gp.x, gp.y - rng, 0, 0)); // BOTTOM
    ymin = std::lower_bound(agents_byy_vec.begin(), agents_byy_vec.end(), &edge, lty());

    edge.set_pos(Pose(gp.x, gp.y + rng, 0, 0)); // TOP
    ymax = std::upper_bound(agents_byy_vec.begin(), agents_byy_vec.end(), &edge, lty());

    // put these models into sets keyed on pointer
    std::set<Agent *> horiz, vert;

    for (; xmin != xmax; ++xmin)
        horiz.insert(*xmin);

    for (; ymin != ymax; ++ymin)
        vert.insert(*ymin);

    // the intersection of the sets is all the fiducials close by
    std::vector<Agent *> nearby;
    std::set_intersection(horiz.begin(), horiz.end(), vert.begin(), vert.end(), std::inserter(nearby, nearby.end()));

    // now test more carefully for whether these neighbors are in agent's FOV
    int num_nearby = nearby.size();
    for (int i = 0; i < num_nearby; i++) {
        Pose nbr_pos = nearby[i]->get_pos();
        int nbr_id = nearby[i]->id;

        cone_result cr = in_vision_cone(agent_pos, nbr_pos, sp->sensing_range, sp->sensing_angle);
        if (cr.in_cone && agent_id != nbr_id) {
            sensor_result new_result;
            new_result.dist_away = cr.dist_away;
            new_result.id = nbr_id;
            result.push_back(new_result);
        }
    }

    return result;
}

bool SimulationData::vecs_sorted() {
    bool sorted = true;

    const char* redText = "\033[1;31m";
    const char* resetText = "\033[0m";

    Agent *begin_agent = *agents_byx_vec.begin();
    double last_x = begin_agent->get_pos().x;
    for (Agent *a : agents_byx_vec) {
        if(!(last_x <= a->get_pos().x)) {
            printf("%sx-positions not sorted in vec!%s\n", redText, resetText);
            sorted = false;
        }
        last_x = a->get_pos().x;
    }

    begin_agent = *agents_byy_vec.begin();
    double last_y = begin_agent->get_pos().y;
    for (Agent *a : agents_byx_vec) {
        if(!(last_y <= a->get_pos().y)) {
            printf("%sy-positions not sorted in vec!%s\n", redText, resetText);
            sorted = false;
        }
        last_x = a->get_pos().y;
    }

    return sorted;
}