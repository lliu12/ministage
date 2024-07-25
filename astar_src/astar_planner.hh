#ifndef ASTAR_PLANNER
#define ASTAR_PLANNER

#include "../shared_utils.hh"
#include "astar_utils.hh"
#include <unordered_set>

class AStarPlanner {
    public:
    // 2D reservation table (for obstacles that are always there)
    std::vector<std::vector<bool>> permanent_reservations;

    // 3D reservation table
    std::vector<std::vector<std::vector<bool>>> reservations;
    SpaceDiscretizer *space;
    bool connect_diagonals;
    int total_timesteps;
    int *timestep; // current time (pointer to sim manager variable)

    // datatype for storing relevant information
    struct Node {
        // f = g + h
        double f, g; // internal scores used in a* planning
        int t; // time
        SiteID pos;
        SiteID parent;
        
        // constructor
        Node(SiteID my_pos, SiteID my_parent, int timestep, double f_val, double g_val)
            : pos(my_pos), parent(my_parent), t(timestep), f(f_val), g(g_val)
        {}

        Node() {}
    };


    // Constructor
    AStarPlanner(SpaceDiscretizer *sim_space, bool diags, int time_steps, int *t);

    // Destructor
    ~AStarPlanner();

    // Distance heuristic
    float dist_heuristic(SiteID a, SiteID b);

    // 2D search
    // delete this function after reservation table is upgraded to 3D
    std::vector<SiteID> search_2d(SiteID start, SiteID goal);

    // 3D search
    std::vector<SiteID> search(SiteID start, SiteID goal, meters_t sensing_range = 0, radians_t sensing_angle = 0);

    // check if anything occupies the sensing cone in Pose p at time t
    bool sensing_cone_occupied(SiteID sensing_from, radians_t a, int t, meters_t sensing_range = 0, radians_t sensing_angle = 0);

    // void reset();
    void clear_reservations();
    
    // recover plan from the data generated during a search
    std::vector<SiteID> recover_plan(SiteID start, SiteID goal,  std::vector<std::vector<std::vector<Node>>> *node_details, int goal_reached_time);

    // recover plan from the data generated during a search
    std::vector<SiteID> recover_plan_2d(SiteID start, SiteID goal, std::vector<std::vector<Node>> *node_details);
};


#endif