#ifndef AGENTS_H
#define AGENTS_H

#include <cmath>
#include "random.hh"
#include "utils.hh"

// An agent with sensing abilities and a location
class Agent {
    public:
    int id; 
    sim_params sp;
    SimulationData *sd;
    Pose *cur_pos; // only this one is a pointer because we want to be able to access this value from SimulationData
    // todo: update *cur_pos to be a smart pointer
    // Pose start_pos;
    Pose goal_pos;
    radians_t travel_angle;

    double fwd_speed; // meters per second
    double turn_speed; // radians per second

    int goals_reached;
    uint64_t goal_birth_time;
    // closest = NULL
    bool stop = 0;

    //// Use rejection sampling to get a random point in a the ring or square between radius r_lower and r_upper (center at origin)
    Pose random_goal();

    //// Set up start and goal positions
    virtual void gen_start_goal_positions();

    // //// Set up waypoint storage (used to visualize next goal)
    // virtual void gen_waypoint_data();

    //// Reset robot data for a new trial
    virtual void reset();

    //// Updates to make when robot reaches goal
    virtual void goal_updates();

    //// Updates made each step for values other than robot speed and direction
    //// Determine whether robot is blocked and update info about blockedness, closest neighbor, periodicity updates, etc.
    virtual void sensing_update();

    //// Update robot position
    virtual void position_update();

    //// Get (global) angle robot should move in to head straight to goal
    double angle_to_goal();

    /// Update current position
    void set_pos(Pose p);


    // Constructor
    Agent(int agent_id, sim_params sim_params, SimulationData *sim_data, Pose *cur_pos_ptr);

    // Destructor
    ~Agent();
};


#endif