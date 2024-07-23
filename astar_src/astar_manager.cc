#include "astar_manager.hh"

// Constructor
AStarManager::AStarManager(sim_params sim_params) {
    sp = sim_params;
    timestep = 0;

    // Discretize space into sites
    sp.cell_width = 2.0 * sp.r_upper / sp.cells_per_side;
    space = new SpaceDiscretizer(sp.r_upper, sp.cells_per_side, sp.periodic, sp.diags);

    // Create planner
    planner = new AStarPlanner(space, sp.diags, sp.time_steps, &timestep);

    // Create agents
    for (int i = 0; i < sp.num_agents; i++) {
        agents.push_back(new AStarAgent(i, &sp, space, planner));
    }


}


// Destructor
AStarManager::~AStarManager(){
    delete space;
    for (AStarAgent *a : agents) { delete a; }
}


void AStarManager::update() {
    for (AStarAgent *a : agents) { 
        a->update(); 
        // TODO: UPDATE RESERVATION TABLE FOR THIS AGENT? 
    }
    timestep++;
}

void AStarManager::reset() {
    timestep = 0;
    for (AStarAgent *a : agents) { 
        a->reset(); 
    }


    planner->clear_reservations();
}

