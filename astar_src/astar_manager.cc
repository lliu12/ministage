#include "astar_manager.hh"


// Constructor
AStarManager::AStarManager(sim_params sim_params) {
    sp = sim_params;

    sp.cell_width = 2.0 * sp.r_upper / sp.cells_per_side;

    space = new SpaceDiscretizer(sp.r_upper, sp.cells_per_side, sp.periodic, sp.diags);



}


// Destructor
AStarManager::~AStarManager(){
    delete space;
}


void AStarManager::update() {

}
