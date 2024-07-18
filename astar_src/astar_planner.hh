#ifndef ASTAR_PLANNER
#define ASTAR_PLANNER

#include "../shared_utils.hh"
#include "astar_utils.hh"

class AStarPlanner {
    public:
    std::vector<std::vector<bool>> reservations;
    SpaceDiscretizer *space;
    bool connect_diagonals;

    // // 3D RESERVATION TABLE
    // std::vector<std::vector<bool>> reservations;

    // Constructor
    AStarPlanner(SpaceDiscretizer *sim_space, bool diags);

    // Destructor
    ~AStarPlanner();

    // Distance heuristic
    float dist_heuristic(SiteID a, SiteID b);

    // 2D search
    // delete this function after reservation table is upgraded to 3D
    std::vector<SiteID> search_2d(SiteID start, SiteID goal);

    // 3D search
    std::vector<SiteID> search(SiteID start, SiteID goal);

    // void reset();
    void clear_reservations();
};


#endif