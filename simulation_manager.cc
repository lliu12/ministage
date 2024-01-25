#include "ministage.hh"

// Constructor
SimulationManager::SimulationManager(sim_params sim_params) {
    sp = sim_params;
    sd = new SimulationData(&sim_params);

    // Create agents
    for (int i = 0; i < sp.num_agents; i++) {
        agents.push_back(new Agent(i, &sp, sd, sd->positions[i]));
    }
}

// Destructor
SimulationManager::~SimulationManager(void){}


void SimulationManager::update() {
    // update simtime and the sorted agent info in simulationdata
    sd->update(agents);

    // update all agent sensors
    for (Agent *a : agents) {
        a->sensing_update();
    }

    // update all agent positions
    // working with the assumption that they will not collide due to sufficient stop conditions
    for (Agent *a : agents) {
        a->position_update();
    }
}


void SimulationManager::reset() {
    sd->reset(); // needs to happen first so that sim_time gets reset to 0
    for (int i = 0; i < sp.num_agents; i++) {
            agents[i]->reset();
    }
}


void SimulationManager::run_trials(int trials, double trial_length) {
    for (int i = 0; i < trials; i++) {
        run_trial(trial_length);
    }
}

void SimulationManager::run_trial(double trial_length) {
    reset();
    while (sd->sim_time < trial_length) {
        update();
    }
}