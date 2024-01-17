#include "ministage.hh"

// Constructor
SimulationManager::SimulationManager(sim_params sim_params) {
    
    sp = sim_params;
    sd = SimulationData(sim_params);

    // Create agents
    for (int i = 0; i < sp.num_agents; i++) {
        agents.push_back(new Agent(i, sp, &sd, sd.positions[i]));
    }
}

// Destructor
SimulationManager::~SimulationManager(void){}


void SimulationManager::update() {

}


void SimulationManager::reset() {
    sd.reset();
    for (int i = 0; i < sp.num_agents; i++) {
            agents[i]->reset();
    }
}


void SimulationManager::runTrials(int trials) {
    for (int i = 0; i < trials; i++) {
        runTrial();
    }
}

void SimulationManager::runTrial() {
    reset();
}