#include "astar_manager.hh"

// Constructor
AStarManager::AStarManager(sim_params sim_params) {
    sp = sim_params;
    timestep = 0;

    // Discretize space into sites
    sp.cell_width = 2.0 * sp.r_upper / sp.cells_per_side;
    space = new SpaceDiscretizer(sp.r_upper, sp.cells_per_side, sp.periodic, sp.diags);

    // Create planner
    planner = new AStarPlanner(space, sp.diags_take_longer, sp.time_steps, &timestep, sp.verbose);

    // Create agents
    for (int i = 0; i < sp.num_agents; i++) {
        agents.push_back(new AStarAgent(i, &sp, space, planner));
    }

    // Pass pointer to planner
    planner->agents = &agents;

}


// Destructor
AStarManager::~AStarManager(){
    delete space;
    for (AStarAgent *a : agents) { delete a; }
}


void AStarManager::update() {
    for (AStarAgent *a : agents) { 
        a->update_plan(); 
    }

    for (AStarAgent *a : agents) { 
        a->update_motion(); 
    }

    timestep += sp.diags_take_longer ? 0.5 : 1.0;

    if (sp.verbose) {
        printf("Just updated timestep to %f\n", timestep);
    }
    
    for (AStarAgent *a : agents) { 
        a->update_travel_angle(); // for agents that already have a plan 
    }
    if (sp.verbose) {
        printf("now displaying simulation step %f\n", timestep);
    }


}

void AStarManager::reset() {
    timestep = 0;
    planner->clear_reservations();
    for (AStarAgent *a : agents) { 
        a->reset(); 
    }
}


void AStarManager::run_trials(int trials, double trial_length) {
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


void AStarManager::run_trial(double trial_length, int trial_id) {
    reset();

    while (timestep < trial_length) {
        if (!sp.outfile_name.empty() && fmod(timestep, sp.save_data_interval) < 0.0001) {
            save_data(trial_id);
        }

        update();
    }

    if (!sp.outfile_name.empty()) { save_data(trial_id); }
}


void AStarManager::save_data(int trial_id) {
    for (AStarAgent *a : agents) {
        outfile << std::to_string(trial_id) + std::string(",") +
            std::to_string(sp.periodic) + std::string(",") +
            std::to_string(sp.num_agents) + std::string(",")
            << timestep << std::string(",") +
            std::to_string(a->id) + std::string(",")
            << a->get_pos().idx << std::string(",")
            << a->get_pos().idy << std::string(",") +
            std::to_string(a->goal_birth_time) + std::string(",") +
            std::to_string(a->goals_reached) + std::string(",") +
            sp.addtl_data + std::string(",")
            << std::endl;
    }
}
