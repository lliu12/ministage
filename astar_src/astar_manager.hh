#ifndef ASTAR_MANAGER
#define ASTAR_MANAGER

// #include <algorithm>
#include <cmath>
#include <set>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <stdlib.h>
#include <string.h>
#include <iomanip>

#include "../random.hh"
#include "astar_utils.hh"
#include "../shared_utils.hh"


// A simulation instance
class AStarManager {
    public:
    // Constructor
    AStarManager(sim_params sim_params);
    // Destructor
    ~AStarManager();

    sim_params sp;

    // 2D vector of pointers to sites
    // std::vector<std::vector<Site *>> sites;

    SpaceDiscretizer *space;

    // SimulationData *sd;
    // /** Pointers to all the agents in this world. */
    // std::vector <Agent *> agents;
    // std::ofstream outfile;

    void update();
    void reset();
    void run_trials(int trials, double trial_length);
    void run_trial(double trial_length, int trial_id);
    void save_data(int trial_id);
};


#endif