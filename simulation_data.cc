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
    // Poses should be reset by the agents in Agent::reset()
}

// Return who an agent with id agent_id and Pose agent_pos would sense in its cone-shaped field of view
std::vector <sensor_result> SimulationData::sense(int agent_id, Pose agent_pos, meters_t sensing_range, radians_t sensing_angle) {
    std::vector <sensor_result> result;

    for (int i = 0; i < sp.num_agents; i++) {
        Pose nbr_pos = *positions[i];
        cone_result cr = in_vision_cone(agent_pos, nbr_pos, sp.sensing_range, sp.sensing_angle);
        if (cr.in_cone && agent_id != i) {
            sensor_result new_result;
            new_result.dist_away = cr.dist_away;
            new_result.id = i;

            result.push_back(new_result);
        }
    }

    return result;
}