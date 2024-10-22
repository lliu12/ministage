#ifndef ASTAR_PLANNER
#define ASTAR_PLANNER

#include "../shared_utils.hh"
#include "astar_utils.hh"
#include <unordered_set>
#include <unordered_map>
#include <thread>   // for std::this_thread::sleep_for
#include <chrono>   // for std::chrono::seconds

class AStarAgent;

class AStarPlanner {
    public:
    // 2D reservation table (for obstacles that are always there)
    std::vector<std::vector<bool>> permanent_reservations;

    // Store pointers to agents so we can call replans
    std::vector <AStarAgent *> *agents;

    // datatype storing a reservation
    struct Reservation {
        float t;
        int idx;
        int idy;
        Reservation(float time, int x, int y) : t(time), idx(x), idy(y)
        {
            if (fmod(t, 0.5) > 1e-5) { printf("Error: Reservations only support times t which are multiples of 0.5."); }
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

        bool operator==(const Reservation &r) const { return (idx == r.idx && idy == r.idy && t == r.t); }
    };

    // reservation table as hashmap
    // std::unordered_set<Reservation, Reservation::hash> reservations;
    std::unordered_map<Reservation, int, Reservation::hash> reservations; 


    SpaceDiscretizer *space;
    bool connect_diagonals;
    bool diags_take_longer; // if true, diagonals take 3 0.5-length timesteps, while adjacents only take 2
    int total_timesteps;
    float *timestep; // current time (pointer to sim manager variable)
    bool verbose;


    struct Node {
        // f = g + h
        double f, g; // internal scores used in a* planning
        float t; // time we arrive at pos
        SiteID pos; // SiteID of this location
        SiteID parent; // SiteID of previous location
        
        // constructor
        Node(SiteID my_pos, SiteID my_parent, float timestep, double f_val, double g_val)
            : pos(my_pos), parent(my_parent), t(timestep), f(f_val), g(g_val)
        {}

        Node() {}
    };


    // Constructor
    AStarPlanner(SpaceDiscretizer *sim_space, bool slower_diags, int time_steps, float *t, bool v);

    // Destructor
    ~AStarPlanner();

    // Distance heuristic
    float dist_heuristic(SiteID a, SiteID b);

    // 2D search
    // delete this function after reservation table is upgraded to 3D
    std::vector<SiteID> search_2d(SiteID start, SiteID goal);

    // 3D search
    std::vector<SiteID> search(SiteID start, SiteID goal, meters_t sensing_range = 0, radians_t sensing_angle = 0, int agent_id = -1);

    // check if a step is valid
    // cur_t is the time we arrived at SiteID cur
    // nbr is the location we are considering moving to next
    bool is_invalid_step(SiteID cur, SiteID nbr, float cur_t, meters_t sensing_range, radians_t sensing_angle);

    // check if anything occupies the sensing cone in Pose p at time t
    bool sensing_cone_occupied(SiteID sensing_from, radians_t a, float t, meters_t sensing_range = 0, radians_t sensing_angle = 0);

    // check if anything occupies the sensing cone in Pose p at time t
    // detect if agent senses another occupied site OR is within view of another agent
    // a indicates the robot's heading; use 1 to run the function and check only if the robot is blocking others
    bool sensing_cone_invalid(SiteID sensing_from, radians_t a, float t, meters_t sensing_range = 0, radians_t sensing_angle = 0, bool verbose = false);

    // return id's of neighbors we are blocking
    std::unordered_set<int> robots_we_block(SiteID sensing_from, float t, meters_t sensing_range, radians_t sensing_angle, bool verbose = false);

    // void reset();
    void clear_reservations();
    
    // recover plan from the data generated during a search
    std::vector<SiteID> recover_plan(SiteID start, SiteID goal,  std::unordered_map<Reservation, Node, Reservation::hash> *node_details, float goal_reached_time, int agent_id);

    void make_reservation(float t, int idx, int idy, int agent_id) {
        if (reserved(t, idx, idy)) {
            int blocker_id = reservations[Reservation(t, idx, idy)];
            printf("\033[31mError: This reservation for time %f, pos %i, %i is already reserved by agent %i! \n\033[0m", t, idx, idy, blocker_id);
            // Pause execution for 10 seconds
            // std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        // reservations.insert(Reservation(t, idx, idy)); // make reservation
        reservations[Reservation(t, idx, idy)] = agent_id;
        if (verbose) {
            printf("Reservation: time %f, pos %i, %i \n", t, idx, idy); // print information
        }
    }

    bool reserved(float t, int idx, int idy) {
        return (reservations.find(Reservation(t, idx, idy)) != reservations.end());
    }
};


#endif