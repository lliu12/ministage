#include "utils.hh"
#include "agents.hh"

// Constructor
SimulationData::SimulationData(sim_params *sim_params) {
    sp = sim_params;
    sim_time = 0;
}

// Destructor
SimulationData::~SimulationData(){}

// Reset
void SimulationData::reset() {
    sim_time = 0;
    // don't need to clear the byx, byy vectors because they store agent pointers and the agents reset themselves
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
void SimulationData::update(std::vector <Agent *> agents) {
    // update simulation time
    sim_time += sp->dt;

    // update for vectors
    std::sort(agents_byx_vec.begin(), agents_byx_vec.end(), ltx());
    std::sort(agents_byy_vec.begin(), agents_byy_vec.end(), lty());
}


// Return who an agent with id agent_id and Pose agent_pos would sense in its cone-shaped field of view
std::vector <sensor_result> SimulationData::sense(int agent_id, Pose agent_pos, meters_t sensing_range, radians_t sensing_angle) {
    std::vector <sensor_result> result;

    // std::cout << "\nTesting fiducial sorting in sensor function..." << std::endl;

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