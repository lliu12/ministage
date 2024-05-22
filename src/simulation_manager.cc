#include "simulation_manager.hh"

// Constructor
SimulationManager::SimulationManager(sim_params sim_params) {
    sp = sim_params;

    // Warnings about incompatible parameter settings
    if(sp.periodic & sp.use_cell_lists & (sp.cells_range != sp.r_upper)) { 
        sp.cells_range = sp.r_upper; 
        if (sp.verbose){ 
            printf("Warning: in a periodic simulation using cell lists, cells_range must equal r_upper range.\n");
        }
    }

    // Derived parameters
    sp.cells_per_side = floor(2.0 * sp.cells_range / sp.sensing_range);
    sp.cell_width = 2.0 * sp.cells_range / sp.cells_per_side;

    // Initialize Simulation Data
    sd = new SimulationData(&sp);

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
SimulationManager::~SimulationManager(){
    delete sd;
    for (Agent *a : agents) { delete a; }
}


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

    // Set up outfile for saving data
    if (!sp.outfile_name.empty()) {
        outfile << std::fixed << std::setprecision(2);
        outfile.open(sp.outfile_name, std::ios_base::app);
    }

    for (int i = 0; i < trials; i++) {
        run_trial(trial_length, i);
    }

    // close outfile
    if (!sp.outfile_name.empty()) { outfile.close(); }
    
}

void SimulationManager::run_trial(double trial_length, int trial_id) {
    reset();
    while (sd->sim_time < trial_length) {

        if (!sp.outfile_name.empty() && fmod(sd->sim_time, sp.save_data_interval) < 0.0001) {
            save_data(trial_id);
        }

        update();
    }

    if (!sp.outfile_name.empty()) { save_data(trial_id); }
}




void SimulationManager::save_data(int trial_id) {
    for (Agent *aa : agents) {
        GoalAgent *a = (GoalAgent *)aa; // cast to GoalAgent
        if (!a->stop) {
            outfile << std::to_string(trial_id) + std::string(",") +
                std::to_string(sp.periodic) + std::string(",") +
                std::to_string(sp.num_agents) + std::string(",")
                << sp.anglenoise << std::string(",")
                << sp.noise_prob << std::string(",")
                << sd->sim_time << std::string(",") +
                std::to_string(a->id) + std::string(",")
                << a->get_pos().x << std::string(",")
                << a->get_pos().y << std::string(",")
                << a->get_pos().a << std::string(",") +
                std::to_string(a->goal_birth_time) + std::string(",") +
                std::to_string(a->goals_reached) + std::string(",") +
                std::to_string(a->stop) + std::string(",") +
                std::to_string(-1) + std::string(",") +
                sp.addtl_data + std::string(",")
                << std::endl;
        }

        else {
            if (a->sensed.size() == 0) {
                printf("Error: Robot stopped but nothing in fiducials.... \n");
                printf("Sim time: %f, Robot ID: %i \n", sd->sim_time, a->id);
            }

            for (sensor_result other : a->sensed) {
                    outfile << std::to_string(trial_id) + std::string(",") +
                        std::to_string(sp.periodic) + std::string(",") +
                        std::to_string(sp.num_agents) + std::string(",")
                        << sp.anglenoise << std::string(",")
                        << sp.noise_prob << std::string(",")
                        << sd->sim_time << std::string(",") +
                        std::to_string(a->id) + std::string(",")
                        << a->get_pos().x << std::string(",")
                        << a->get_pos().y << std::string(",")
                        << a->get_pos().a << std::string(",") +
                        std::to_string(a->goal_birth_time) + std::string(",") +
                        std::to_string(a->goals_reached) + std::string(",") +
                        std::to_string(a->stop) + std::string(",") +
                        std::to_string(other.id) + std::string(",") +
                        sp.addtl_data + std::string(",")
                        << std::endl;

            }
        }
    }
}