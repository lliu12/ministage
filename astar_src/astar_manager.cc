#include "astar_manager.hh"


// Constructor
AStarManager::AStarManager(sim_params sim_params) {
    sp = sim_params;

    sp.site_dist = 2.0 * sp.r_upper / sp.sites_per_side;


    // abstract this copy pasting away!!!


    // // generate sites
    // float cur_x = -1.0 * sp.r_upper;
    // for (int idx = 0; idx < sp.sites_per_side; idx++) {
    //     float cur_y = -1.0 * sp.r_upper;
    //     for (int idy = 0; idy < sp.sites_per_side; idy++) {
    //         sites[idx][idy] = new Site( cur_x, cur_y, sp.site_dist);
    //         sites[idx][idy]->is_outer = (idx == 0 || idy == 0 || 
    //                                         idx == sp.sites_per_side - 1 || idy == sp.sites_per_side - 1);

    //         cur_y += sp.site_dist;
    //     }
    //     cur_x += sp.site_dist;
    // }

    // populate each site's vector of neighbors

}


// Destructor
AStarManager::~AStarManager(){
}
