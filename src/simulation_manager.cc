#include "simulation_manager.hh"

// Constructor
SimulationManager::SimulationManager(sim_params sim_params) {
    sp = sim_params;

    // Warnings about incompatible parameter settings
    if(sp.periodic & sp.use_cell_lists & (sp.cells_range != sp.r_upper)) { 
        sp.cells_range = sp.r_upper; 
        printf("Warning: in a periodic simulation using cell lists, cells_range must equal r_upper range.\n");
    }



    sd = new SimulationData(&sim_params);

    // Create agents vector and pass copies to SimulationData
    for (int i = 0; i < sp.num_agents; i++) {
        agents.push_back(new NoiseAgent(i, &sp, sd));
    }

    sd->agents = agents;

    if (sp.use_sorted_agents) {
        sd->agents_byx_vec = agents;
        sd->agents_byy_vec = agents;
    }

    sd->reset();
}

// Destructor
SimulationManager::~SimulationManager(void){}


void SimulationManager::update() {
    // save data here before any updates occur

    // update simtime and the sorted agent info in simulationdata
    sd->update();

    // update all agent sensors
    for (Agent *a : agents) { a->sensing_update(); }

    // update all agent positions
    // working with the assumption that they should not collide in this one step due to sufficient stop conditions
    for (Agent *a : agents) { a->position_update(); }

}


void SimulationManager::reset() {
    sd->sim_time = 0; // needs to happen first since agents store this time as goal_birth_time
    for (Agent *a : agents) { a->reset(); }
    sd->reset(); // new randomized poses are out of order... sort them again!
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