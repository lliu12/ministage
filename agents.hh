#ifndef AGENTS_H
#define AGENTS_H

#include <cmath>
#include "random.hh"
#include "utils.hh"

class SimulationData;

// Base Agent class
// An agent with sensing abilities and a location
class Agent {
    public:
    Pose goal_pos;
    int id;
    sim_params *sp;
    SimulationData *sd;
    Pose *cur_pos;
    Color color;

    // current speeds
    double fwd_speed; // meters per second
    double turn_speed; // radians per second

    virtual void reset();

    // Update sensor information
    virtual void sensing_update();

    // Update the robot's intended forward and turning speed
    virtual void decision_update();

    // Use forward and turning speed to update robot position
    virtual void position_update();

    /// Update current position
    void set_pos(Pose p);

    /// Update current position
    Pose get_pos() const;

    //// Use rejection sampling to get a random point in a the ring or square between radius r_lower and r_upper (center at origin)
    Pose random_pos();

    virtual void draw();

    // Constructor
    Agent(int agent_id, sim_params *sim_params, SimulationData *sim_data);
    Agent();

    // Destructor
    ~Agent();
};


// A robot which navigates directly to randomly generated individual goals
class GoalAgent : public Agent {
    public:
    // Pose goal_pos;
    radians_t travel_angle;

    int goals_reached;
    uint64_t goal_birth_time;
    bool stop = 0;

    // //// Set up waypoint storage (used to visualize next goal)
    // virtual void gen_waypoint_data();

    //// Reset robot data for a new trial
    virtual void reset() override;

    //// Updates to make when robot reaches goal
    virtual void goal_updates();

    // Update the robot's intended forward and turning speed
    virtual void sensing_update() override;

    // Update the robot's intended forward and turning speed
    virtual void decision_update() override;

    //// Get (global) angle robot should move in to head straight to goal
    virtual double angle_to_goal();

    virtual void draw() override;

    // Constructor
    GoalAgent(int agent_id, sim_params *sim_params, SimulationData *sim_data);
    GoalAgent();

    // Destructor
    ~GoalAgent();
};


// A robot which navigates to randomly generated individual goals, sometimes adding noise to its motion
class NoiseAgent : public GoalAgent {
    public:
    int current_phase_count, runsteps; // time so far spent running or tumbling, total length of a run or tumble period
    double travel_angle; // amount of noise and bias to add to random angles

    //// Constructor
    NoiseAgent(int agent_id, sim_params *sim_params, SimulationData *sim_data);
    NoiseAgent();

    //// Destructor
    ~NoiseAgent();

    //// Reset robot data for a new trial
    virtual void reset() override;

    // // Update the robot's intended forward and turning speed
    // virtual void sensing_update() override;

    // Update the robot's intended forward and turning speed
    virtual void decision_update() override;

    //// Determine angle for robot to steer in (after adding noise)
    virtual double get_travel_angle();

    virtual void goal_updates() override;

};


#endif