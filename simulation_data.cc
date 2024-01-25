#include "utils.hh"
#include "agents.hh"

// Constructor
SimulationData::SimulationData(sim_params *sim_params) {
    sp = sim_params;

    // Set up Pose pointers
    for (int i = 0; i < sp->num_agents; i++) {
        positions.push_back(new Pose());
    }

    // Set up sim_time pointer
    sim_time = 0;
}

// Destructor
SimulationData::~SimulationData(){}

// Reset
void SimulationData::reset() {
    sim_time = 0;
    // Poses should be reset by the agents in Agent::reset()
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

    // update sorted agent positions
    agents_byx.clear();
    agents_byy.clear();

    for (Agent *a : agents) {
        agents_byx.insert(a);
        agents_byy.insert(a);
    }

}


// Return who an agent with id agent_id and Pose agent_pos would sense in its cone-shaped field of view
std::vector <sensor_result> SimulationData::sense(int agent_id, Pose agent_pos, meters_t sensing_range, radians_t sensing_angle) {
    std::vector <sensor_result> result;

    // std::cout << "\nTesting fiducial sorting in sensor function..." << std::endl;
    // bool sorted = sets_sorted();
    
    // // std::cout << "Testing narrowing down nearby neighbors..." << std::endl;
    // Pose dummy_pose;
    // Agent edge(-1, sp, this, &dummy_pose); // dummy model used to find bounds in the sets

    // Pose gp = agent_pos;
    // sp->.set_pos(Pose(gp.x - sp->sensing_range, gp.y, 0, 0)); // LEFT
    // std::set<Agent *, SimulationData::ltx>::iterator xmin = agents_byx.lower_bound(&sp->);
    // // Agent *xminAgent = *xmin;
    // if(xmin == agents_byx.end()) {
    //     printf("left-filtered set empty for agent %i \n", agent_id);
    // }
    // }




    // ------- above this is debugging


    // first, find a smaller collection of nearby neighbors
    double rng = sp->sensing_range;
    Pose gp = agent_pos;
    Pose dummy_pose;
    Agent edge(-1, sp, this, &dummy_pose); // dummy model used to find bounds in the sets

    edge.set_pos(Pose(gp.x - rng, gp.y, 0, 0)); // LEFT
    std::set<Agent *, ltx>::iterator xmin =
        agents_byx.lower_bound(&edge); // O(log(n))

    // printf("\n\n looking inside left-bounded set... \n");
    // edge.get_pos().Print("dummy pose currently: ");
    // for (Agent *a : xmin) {
    //     a->get_pos().Print("");
    // }


    edge.set_pos(Pose(gp.x + rng, gp.y, 0, 0)); // RIGHT
    // edge.get_pos().Print("dummy pose currently: ");
    const std::set<Agent *, ltx>::iterator xmax =
        agents_byx.upper_bound(&edge);

    edge.set_pos(Pose(gp.x, gp.y - rng, 0, 0)); // BOTTOM
    std::set<Agent *, ltx>::iterator ymin =
        agents_byy.lower_bound(&edge);

    edge.set_pos(Pose(gp.x, gp.y + rng, 0, 0)); // TOP
    const std::set<Agent *, ltx>::iterator ymax =
        agents_byy.upper_bound(&edge);

    // put these models into sets keyed on pointer
    std::set<Agent *> horiz, vert;

    for (; xmin != xmax; ++xmin)
        horiz.insert(*xmin);

    for (; ymin != ymax; ++ymin)
        vert.insert(*ymin);

    // the intersection of the sets is all the fiducials close by
    std::vector<Agent *> nearby;
    std::set_intersection(horiz.begin(), horiz.end(), vert.begin(), vert.end(), std::inserter(nearby, nearby.end()));

    // printf("\n\n neighbors with horizontal position near agent %i... \n", agent_id);
    // agent_pos.Print("Agent pose: ");
    // if (xmin != agents_byx.end()) {
    //     Agent* xminAgent = *xmin;
    //     printf("agent id: %i \n", xminAgent->id);
    //     xminAgent->get_pos().Print("Left bound agent's position"); // sometimes this segfaults...
    // }
    // else {
    //     printf("xmin is past end of set\n");
    // }

    // for (Agent *a : horiz) {
    //     a->get_pos().Print("neighbor: ");
    // }
    


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

            // // printf("blocker id: %i", nbr_id);
            // nbr_pos.Print("I am nearby!");

            result.push_back(new_result);
        }
    }


    // working but slow code
    // for (int i = 0; i < sp->num_agents; i++) {
    //     Pose nbr_pos = *positions[i];
    //     cone_result cr = in_vision_cone(agent_pos, nbr_pos, sp->sensing_range, sp->sensing_angle);
    //     if (cr.in_cone && agent_id != i) {
    //         sensor_result new_result;
    //         new_result.dist_away = cr.dist_away;
    //         new_result.id = i;

    //         result.push_back(new_result);

    //         nbr_pos.Print("I am nearby!");
    //     }
    // }

    return result;
}

bool SimulationData::sets_sorted() {
    bool sorted = true;

    Agent *begin_agent = *agents_byx.begin();
    double last_x = begin_agent->get_pos().x;
    for (Agent *a : agents_byx) {
        if(!(last_x <= a->get_pos().x)) {
            // ANSI escape code for red text
            const char* redText = "\033[1;31m";

            // Reset ANSI escape code to default
            const char* resetText = "\033[0m";

            // Your text goes here
            printf("%sx-positions not sorted!%s\n", redText, resetText);


            sorted = false;
        }
        last_x = a->get_pos().x;
    }

    begin_agent = *agents_byy.begin();
    double last_y = begin_agent->get_pos().y;
    for (Agent *a : agents_byx) {
        if(!(last_y <= a->get_pos().y)) {
            // ANSI escape code for red text
            const char* redText = "\033[1;31m";

            // Reset ANSI escape code to default
            const char* resetText = "\033[0m";

            // Your text goes here
            printf("%sy-positions not sorted!%s\n", redText, resetText);


            sorted = false;
        }
        last_x = a->get_pos().y;
    }

    return sorted;
}