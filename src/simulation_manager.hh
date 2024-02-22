#ifndef MINISTAGE_H
#define MINISTAGE_H

// #include <algorithm>
#include <cmath>
#include <set>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <stdlib.h>
#include <string.h>
#include <iomanip>

#include "random.hh"
#include "agents.hh"
#include "utils.hh"


// A simulation instance
class SimulationManager {
    public:
    // Constructor
    SimulationManager(sim_params sim_params);
    // Destructor
    ~SimulationManager();

    sim_params sp;
    SimulationData *sd;
    /** Pointers to all the agents in this world. */
    std::vector <Agent *> agents;
    std::ofstream outfile;

    void update();
    void reset();
    void run_trials(int trials, double trial_length);
    void run_trial(double trial_length, int trial_id);
    void save_data(int trial_id);
};


#endif