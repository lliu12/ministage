#include "utils.hh"

// Constructor
SimulationData::SimulationData() {}
SimulationData::SimulationData(sim_params sim_params) {
    sp = sim_params;

    // Set up Pose pointers
    for (int i = 0; i < sp.num_agents; i++) {
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
}