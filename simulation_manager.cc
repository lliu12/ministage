#include "ministage.hh"

// Constructor
SimulationManager::SimulationManager(sim_params sim_params) {
    sp = sim_params;
    sd = new SimulationData(sim_params);

    // Create agents
    for (int i = 0; i < sp.num_agents; i++) {
        agents.push_back(new Agent(i, sp, sd, sd->positions[i]));
    }
}

// Destructor
SimulationManager::~SimulationManager(void){}


void SimulationManager::update() {
    // update simtime and the sorted agent info in simulationdata
    // std::cout << "in manager: " << sd << std::endl;
    sd->update(agents);
    // std::cout << "\n\n Testing fiducial sorting, right after update ..." << std::endl;
    // bool sorted = sd.sets_sorted();

    // update all agent sensors
    for (int i = 0; i < sp.num_agents; i++) {
        // printf("Testing fiducial sorting, starting loop for agent %i ... \n", i);
        // bool sorted = sd.sets_sorted();

        agents[i]->sensing_update();

        // printf("Testing fiducial sorting, ending loop for agent %i ... \n", i);
        // sorted = sd.sets_sorted();
    }

    // update all agent positions
    // working with the assumption that they will not collide due to sufficient stop conditions
    for (int i = 0; i < sp.num_agents; i++) {
        agents[i]->position_update();
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