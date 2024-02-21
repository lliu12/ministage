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
SimulationData::~SimulationData() {

    for (const auto& row : cells) {
        for (Cell *c : row) {
            delete c;
        }
    }
    delete overflow_cell;
}

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
    for (Agent *a : agents) 
    { 
        if (a->fwd_speed != 0) {
            all_blocked_helper = false;
            break;
        }
    }

    // if anyone has moved, or the sim has just begun, proceed with the full update
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
        if (sp->verbose && !all_stopped) {
            printf("everyone is blocked at time %f\n", sim_time);
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
    overflow_cell->occupants.clear();

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
            // link to neighbor on right
            cell_neighbor_helper(idx, idy, idx + 1, idy);

            // link to neighbor above
            cell_neighbor_helper(idx, idy, idx, idy + 1);

            // link to upper right diagonal neighbor
            cell_neighbor_helper(idx, idy, idx + 1, idy + 1);

            // link to upper left diagonal neighbor
            cell_neighbor_helper(idx, idy, idx - 1, idy + 1);

            // link to overflow cell
            if (cells[idx][idy]->is_outer_cell) {
                // link to overflow cell
                cells[idx][idy]->add_neighbor(overflow_cell);
                overflow_cell->add_neighbor(cells[idx][idy]);
            }
        }
    }
}


// add the two cells as neighbors (two-way) if they are a valid pair (handling periodic cell wrapping)
void SimulationData::cell_neighbor_helper(int idx, int idy, int nbr_idx, int nbr_idy) {
    int cps = sp->cells_per_side;
    // if neighbor cell is beyond cell array bounds, only add them (after wrapping) if simulation is periodic
    if (nbr_idx >= cps || nbr_idx < 0 || nbr_idy >= cps || nbr_idy < 0) {
        if (sp->periodic) {
                Cell *wrapped_nbr = cells[(nbr_idx + cps) % cps][(nbr_idy + cps) % cps];
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

// Find nearby agents to a given position
std::vector<Agent *> SimulationData::find_nearby_sorted_agents(Pose *agent_pos) {
    std::vector<Agent *> nearby_sorted_agents;

    // vecs_sorted(); // test whether the position vectors are sorted
    
    if (sp->use_sorted_agents) {
        double rng = sp->sensing_range;
        Pose *gp = agent_pos;
        Agent edge(-1, sp, this); // dummy model used to find bounds in the sets
        std::vector<Agent *>::iterator xmin, xmax, ymin, ymax;

        edge.set_pos(Pose(gp->x - rng, gp->y, 0, 0)); // LEFT
        xmin = std::lower_bound(agents_byx_vec.begin(), agents_byx_vec.end(), &edge, ltx());

        edge.set_pos(Pose(gp->x + rng, gp->y, 0, 0)); // RIGHT
        xmax = std::upper_bound(agents_byx_vec.begin(), agents_byx_vec.end(), &edge, ltx());

        edge.set_pos(Pose(gp->x, gp->y - rng, 0, 0)); // BOTTOM
        ymin = std::lower_bound(agents_byy_vec.begin(), agents_byy_vec.end(), &edge, lty());

        edge.set_pos(Pose(gp->x, gp->y + rng, 0, 0)); // TOP
        ymax = std::upper_bound(agents_byy_vec.begin(), agents_byy_vec.end(), &edge, lty());

        // put these models into sets keyed on pointer
        std::set<Agent *> horiz, vert;

        for (; xmin != xmax; ++xmin)
            horiz.insert(*xmin);

        for (; ymin != ymax; ++ymin)
            vert.insert(*ymin);

        // the intersection of the sets is all the fiducials close by
        std::set_intersection(horiz.begin(), horiz.end(), vert.begin(), vert.end(), std::inserter(nearby_sorted_agents, nearby_sorted_agents.end()));
    }

    return nearby_sorted_agents;
}

// Use cell lists instead
std::vector<Agent *> SimulationData::find_nearby_cell_lists(Pose *agent_pos) {
    std::vector<Agent *> nearby;

    if (sp->use_cell_lists) {
        Cell *my_cell = get_cell_for_pos(agent_pos);

        // printf("neighbors to this cell: %zu \n", my_cell->neighbors.size());
        
        // put agents in my_cell and its neighbors into nearby
        nearby.insert(nearby.end(), my_cell->occupants.begin(), my_cell->occupants.end());
        for (Cell *nbr : my_cell->neighbors) {
            // printf("in cell lists inserting neighbors... \n");
            nearby.insert(nearby.end(), nbr->occupants.begin(), nbr->occupants.end());
        }
    }

    return nearby;
}


// Return who an agent with id agent_id and Pose agent_pos would sense in its cone-shaped field of view
std::vector <sensor_result> SimulationData::sense(int agent_id, Pose agent_pos) {
    // check that sensing functions agree
    // neighbor_functions_agree(agent_id, agent_pos);

    std::vector <sensor_result> result;

    // find nearby neighbors with vectors
    // first, find a smaller collection of nearby neighbors
    std::vector<Agent *> nearby = sp->use_cell_lists ? find_nearby_cell_lists(&agent_pos) : find_nearby_sorted_agents(&agent_pos);
    // printf("Neighbors nearby: %zu \n", nearby.size());

    // now test more carefully for whether these neighbors are in agent's FOV
    for (Agent *nbr : nearby) {
        // printf("Agent %i has agent %i nearby... \n", agent_id, nbr->id);

        Pose nbr_pos = nbr->get_pos();
        // if periodic world, test if the nearest periodic coordinate is in FOV
        if (sp->periodic) { nbr_pos = nearest_periodic(agent_pos, nbr_pos, sp->r_upper); }

        int nbr_id = nbr->id;

        cone_result cr = in_vision_cone(agent_pos, nbr_pos, sp->sensing_range, sp->sensing_angle);
        if (cr.in_cone && agent_id != nbr_id) {
            sensor_result new_result;
            new_result.dist_away = cr.dist_away;
            new_result.id = nbr_id;
            result.push_back(new_result);
            // printf("someone in vision cone for agent: %i\n", agent_id);
        }
    }

    return result;
}






// functions for checking sensing correctness

// check that the position vectors are sorted
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

// check that the two neighbor-finding implementations agree (find the same number of neighbors in cone)
bool SimulationData::neighbor_functions_agree(int agent_id, Pose agent_pos) {
    if (!sp->use_cell_lists || ! sp->use_sorted_agents) {
        printf("To check if the two sensing implementations agree, use_cell_lists and use_sorted_agents both need to be true. \n");
        return false;
    }

    bool agree = true; 

    std::vector<Agent *> nearby_sa = find_nearby_sorted_agents(&agent_pos);
    std::vector<Agent *> nearby_cl = find_nearby_cell_lists(&agent_pos);

    std::vector<Agent *> seen_sa;
    std::vector<Agent *> seen_cl;

    for (Agent *nbr : nearby_sa) { 
        Pose nbr_pos = nbr->get_pos();
        if (sp->periodic) { nbr_pos = nearest_periodic(agent_pos, nbr_pos, sp->r_upper); }

        cone_result cr = in_vision_cone(agent_pos, nbr_pos, sp->sensing_range, sp->sensing_angle);
        if (cr.in_cone && agent_id != nbr->id) {
            seen_sa.push_back(nbr);
        }
    }


    for (Agent *nbr : nearby_cl) { 
        Pose nbr_pos = nbr->get_pos();
        if (sp->periodic) { nbr_pos = nearest_periodic(agent_pos, nbr_pos, sp->r_upper); }

        cone_result cr = in_vision_cone(agent_pos, nbr_pos, sp->sensing_range, sp->sensing_angle);
        if (cr.in_cone && agent_id != nbr->id) {
            seen_cl.push_back(nbr);
        }
    }

    if(seen_sa.size() != seen_cl.size()) {
        printf("Different number of agents sensed using sorted positions vs. cell lists! \n");
        agree = false;
    }
    // else {
    //     printf("Sensing methods match. \n");
    // }

    return agree;
}