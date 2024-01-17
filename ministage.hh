#ifndef MINISTAGE_H
#define MINISTAGE_H

/** \file ministage.hh
 *  Desc: Header file for the MiniStage library
 */

// C++ Libraries
#include <algorithm>
#include <cmath>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <vector>
#include <fstream>

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
    SimulationData sd;
    /** Pointers to all the agents in this world. */
    std::vector <Agent *> agents;

    void update();
    void reset();
    void runTrials(int trials);
    void runTrial();
};


#endif