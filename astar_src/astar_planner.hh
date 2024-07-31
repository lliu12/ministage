#ifndef ASTAR_PLANNER
#define ASTAR_PLANNER

#include "../shared_utils.hh"
#include "astar_utils.hh"
#include <unordered_set>

class AStarPlanner {
    public:
    // 2D reservation table (for obstacles that are always there)
    std::vector<std::vector<bool>> permanent_reservations;

    // datatype storing a reservation
    struct Reservation {
        int idx;
        int idy;
        float t;
        Reservation(float time, int x, int y) : t(time), idx(x), idy(y)
        {
            if (fmod(t, 0.5) > 0) { printf("Error: Reservations only support times t which are multiples of 0.5."); }
        }

        struct hash
        {
            size_t operator()(const Reservation &r) const
            {
                // we expect t to be a multiple of 0.5, so here we convert it to an int for safer hashing
                size_t tHash = std::hash<int>()((int)(std::round(2 * r.t))); 
                size_t xHash = std::hash<int>()(r.idx) << 1;
                size_t yHash = std::hash<int>()(r.idy) << 2;

                return xHash ^ yHash ^ tHash;
            }
        };


        // Equality comparison
        bool operator==(const Reservation &r) const { return (idx == r.idx && idy == r.idy && t == r.t); }
        // struct equals {
        //     bool operator()(const Reservation &r1, const Reservation &r2) const {
        //         return r1.idx == r2.idx && r1.idy == r2.idy && r1.t == r2.t;
        //     }
        // };
    };

    // reservation table as hashmap
    std::unordered_set<Reservation, Reservation::hash> reservations;



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

    bool reserved(float t, int idx, int idy) {
        return (reservations.find(Reservation(t, idx, idy)) != reservations.end());
    }
};


#endif