#include "agents.hh"

// Constructor
Agent::Agent(int agent_id, sim_params *sim_params, SimulationData *sim_data, Pose *cur_pos_ptr) {
    sp = sim_params;
    sd = sim_data;    
    id = agent_id;
    cur_pos = cur_pos_ptr; // cur_pos now points to the same location as cur_pos_ptr
    reset();
}
Agent::Agent() {}

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
        rand_x = sp->r_upper * 2 * (Random::get_unif_double(0, 1) - .5);
        rand_y = sp->r_upper * 2 * (Random::get_unif_double(0, 1) - .5);
        double dist = Pose(rand_x, rand_y, 0, 0).Distance(Pose(0,0,0,0));
        if (!sp->circle_arena || (dist <= sp->r_upper && dist >= sp->r_lower)) { done = 1; }
    }

    if (sp->verbose) {
    printf("Random Pose (x, y, angle) generated for agent %i is [%.2f %.2f %.2f] \n", id, rand_x, rand_y, rand_a);
    }    

    return Pose(rand_x, rand_y, 0, rand_a);
}

// initialize robot's start and goal positions
void Agent::gen_start_goal_positions() {
    if (sp->verbose) {
    printf("\nGenerating start and goal for robot %i... \n", id);
    }

    goal_pos = random_goal(); // set goal
    set_pos(random_goal());
    goal_birth_time = sd->sim_time;
    
    // start_pos = *cur_pos; // not a pointer - should not change when cur_pos is eventually updated
    // issue: start pos gets reset every time, doesn't store the true start pos. revisit when testing trials method
}


void Agent::reset() {
    // get new start goal locations 
    gen_start_goal_positions();
    goals_reached = 0;
    stop = 0;
    fwd_speed = 0;
    turn_speed = 0;
    travel_angle = 0;
}

// make updates when robot reaches goal (increase goal counters, generate new goal, etc)
void Agent::goal_updates() {
    goal_pos = random_goal();
    goals_reached++;
    goal_birth_time = sd->sim_time;
}

//// Use sensor information to update motion (turning and forward speed)
void Agent::sensing_update() {
    // first, check if robot has reached its goal and update variables accordingly
    if (get_pos().Distance(goal_pos) < sp->goal_tolerance) {
        goal_updates();
    }

    std::vector <sensor_result> sensed = sd->sense(id, get_pos(), sp->sensing_range, sp->sensing_angle);
    stop = sensed.size() > 0; // agent will stop if any neighbor was sensed in vision cone

    travel_angle = angle_to_goal();
    
    fwd_speed = stop ? 0 : sp->cruisespeed;

    // for instantaneous turning, set robot to travel angle
    if (sp->turnspeed == -1) {
      set_pos(Pose(cur_pos->x, cur_pos->y, cur_pos->z, travel_angle));
      turn_speed = 0;
    }
    // for non-instantaneous turning, set turnspeed
    else {
        double a_error = normalize(travel_angle - cur_pos->a);
        turn_speed = sp->turnspeed * a_error;
    }

}

// Function to set new position
void Agent::set_pos(Pose p) {
    *cur_pos = p;
}

// Function to get Pose
Pose Agent::get_pos() const {
    return *cur_pos;
}

//// Update robot position
void Agent::position_update() {
    // find the change of pose due to our forward and turning motions
    const Pose dp(fwd_speed * sp->dt, 0, 0, normalize(turn_speed * sp->dt));

    // the pose we're trying to achieve
    Pose newpose(*cur_pos + dp);
    set_pos(newpose);

    // update location if world is periodic and robot is now out of bounds
    if (sp->periodic) {
        double s = 2 * sp->r_upper;

        if (cur_pos->x < -s/2 || cur_pos->x > s/2 || cur_pos->y < -s/2 || cur_pos->y > s/2) { // if out of bounds
        double x = fmod(cur_pos->x + s/2, s) - s/2;
        double y = fmod(cur_pos->y + s/2, s) - s/2;
        set_pos(Pose(x > -s/2 ? x : x + s, y > -s/2 ? y : y + s, cur_pos->z, cur_pos->a));
        }
    }
}

//// Get (global) angle robot should move in to head straight to goal
double Agent::angle_to_goal() {
      Pose goal_pos_helper; // will be true goal pos if world is not periodic
      if (!sp->periodic) {
        goal_pos_helper = goal_pos;
      }

      // if space is periodic, figure out where robot should move to for shortest path to goal
      else {
        double s = 2 * sp->r_upper;
        double xs [9] = {-s, -s, -s, 0, 0, 0, s, s, s};
        double ys [9] = {-s, 0, s, -s, 0, s, -s, 0, s};
        int closest_pos = 0;
        double closest_dist = std::numeric_limits<double>::infinity();
        for ( int i=0; i<9; i++ ) {
          Pose diff = Pose(xs[i], ys[i], 0, 0);
          Pose test_pos = goal_pos + diff;
          double dist = cur_pos->Distance(test_pos);
          if (dist < closest_dist) {
            closest_dist = dist;
            closest_pos = i;
          }
        }
        goal_pos_helper = Pose(goal_pos.x + xs[closest_pos], goal_pos.y + ys[closest_pos], 0, 0);
      }
      
      double x_error = goal_pos_helper.x - cur_pos->x;
      double y_error = goal_pos_helper.y - cur_pos->y;
      return atan2(y_error, x_error);
}