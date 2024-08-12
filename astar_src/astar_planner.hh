#ifndef ASTAR_PLANNER
#define ASTAR_PLANNER

#include "../shared_utils.hh"
#include "astar_utils.hh"
#include <unordered_set>
#include <unordered_map>
#include <thread>   // for std::this_thread::sleep_for
#include <chrono>   // for std::chrono::seconds


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
    bool diags_take_longer; // if true, diagonals take 3 0.5-length timesteps, while adjacents only take 2
    int total_timesteps;
    float *timestep; // current time (pointer to sim manager variable)


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
    AStarPlanner(SpaceDiscretizer *sim_space, bool slower_diags, int time_steps, float *t);

    // Destructor
    ~AStarPlanner();

    // Distance heuristic
    float dist_heuristic(SiteID a, SiteID b);

    // 2D search
    // delete this function after reservation table is upgraded to 3D
    std::vector<SiteID> search_2d(SiteID start, SiteID goal);

    // 3D search
    std::vector<SiteID> search(SiteID start, SiteID goal, meters_t sensing_range = 0, radians_t sensing_angle = 0);

    // check if a step is valid
    // cur_t is the time we arrived at SiteID cur
    // nbr is the location we are considering moving to next
    bool is_invalid_step(SiteID cur, SiteID nbr, float cur_t, meters_t sensing_range, radians_t sensing_angle);

    // check if anything occupies the sensing cone in Pose p at time t
    bool sensing_cone_occupied(SiteID sensing_from, radians_t a, int t, meters_t sensing_range = 0, radians_t sensing_angle = 0);

    // void reset();
    void clear_reservations();
    
    // recover plan from the data generated during a search
    std::vector<SiteID> recover_plan(SiteID start, SiteID goal,  std::unordered_map<Reservation, Node, Reservation::hash> *node_details, float goal_reached_time);

    void make_reservation(float t, int idx, int idy) {
        if (reserved(t, idx, idy)) {
            printf("\033[31mError: This reservation for time %f, pos %i, %i is already reserved! \n\033[0m", t, idx, idy);
            // Pause execution for 10 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        reservations.insert(Reservation(t, idx, idy)); // make reservation
        printf("Reservation: time %f, pos %i, %i \n", t, idx, idy); // print information
    }

    bool reserved(float t, int idx, int idy) {
        return (reservations.find(Reservation(t, idx, idy)) != reservations.end());
    }
};


#endif