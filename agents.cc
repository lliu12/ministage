#include "agents.hh"

// Constructor
Agent::Agent(int agent_id, sim_params sim_params, SimulationData *sim_data, Pose *cur_pos_ptr) {
    sp = sim_params;
    sd = sim_data;
    id = agent_id;
    cur_pos = cur_pos_ptr; // cur_pos now points to the same location as cur_pos_ptr
    reset();
}

// Destructor
Agent::~Agent(void){}

// Use rejection sampling to obtain a random point in a the ring between radius r_lower and r_upper (center at origin)
// Or, if not in a circular arena, in the square with center at origin and side length 2 * r_upper
Pose Agent::random_goal()
{
    bool done = 0;
    double rand_x;
    double rand_y;
    double rand_a = 2 * M_PI * (Random::get_unif_double(0, 1) - .5);

    while (!done) {
        rand_x = sp.r_upper * 2 * (Random::get_unif_double(0, 1) - .5);
        rand_y = sp.r_upper * 2 * (Random::get_unif_double(0, 1) - .5);
        double dist = Pose(rand_x, rand_y, 0, 0).Distance(Pose(0,0,0,0));
        if (!sp.circle_arena || (dist <= sp.r_upper && dist >= sp.r_lower)) { done = 1; }
    }

    if (sp.verbose) {
    printf("Random Pose (x, y, angle) generated for agent %i is [%.2f %.2f %.2f] \n", id, rand_x, rand_y, rand_a);
    }    

    return Pose(rand_x, rand_y, 0, rand_a);
}

// initialize robot's start and goal positions
void Agent::gen_start_goal_positions() {
    if (sp.verbose) {
    printf("\nGenerating start and goal for robot %i... \n", id);
    }

    goal_pos = random_goal(); // set goal
    *cur_pos = random_goal();
    goal_birth_time = sd->sim_time;
    
    // start_pos = *cur_pos; // not a pointer - should not change when cur_pos is eventually updated
    // issue: start pos gets reset every time, doesn't store the true start pos. revisit when testing trials method
}


void Agent::reset() {
    // get new start goal locations 
    gen_start_goal_positions();
    goals_reached = 0;
    
}

// make updates when robot reaches goal (increase goal counters, generate new goal, etc)
void Agent::goal_updates() {
    goal_pos = random_goal();
    goals_reached++;
    goal_birth_time = sd->sim_time;
}

//// Use sensor information to update motion (turning and forward speed)
void Agent::sensing_update() {
    std::vector <sensor_result> sensed = sd->sense(id, *cur_pos, sp.sensing_range, sp.sensing_angle);
}


//// Update robot position
void Agent::position_update() {

}

//// Get (global) angle robot should move in to head straight to goal
double Agent::angle_to_goal() {
    return 0;
}