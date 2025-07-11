#ifndef ASTAR_AGENT
#define ASTAR_AGENT

#include "../random.hh"
#include "astar_utils.hh"
#include "../shared_utils.hh"
#include <deque>

class AStarPlanner;

class AStarAgent {
    public:

    int id;
    SiteID goal;
    SiteID cur_pos;
    sim_params *sp;
    SpaceDiscretizer *space;
    AStarPlanner *planner;
    Color color;
    int goals_reached;
    float goal_birth_time;
    radians_t travel_angle;

    // store recent poses
    std::deque<SiteID> trail;

    // plan for upcoming steps
    std::vector<SiteID> plan;

    // // store upcoming positions
    // std::unordered_map<float, SiteID> positions; 

    void get_plan(); //  void get_plan(int replan_depth = 0);

    void abort_plan();

    void update_travel_angle();

    void update_plan();

    void update_motion();

    void goal_reached_update();

    void reset();

    void set_pos(SiteID pos);

    SiteID get_pos();

    Pose get_pos_as_pose();

    void update_trail();

    // Return the step agent will take during the simulation step starting at time t. 
    // This function needs to be called before update_motion is called (when the current step is dropped from the plan)
    SiteID step_at_time(float t);

    void draw();

    SiteID random_pos();

    // Constructor
    AStarAgent(int agent_id, sim_params *sim_params, SpaceDiscretizer *sim_space, AStarPlanner *sim_planner);

    // Destructor
    ~AStarAgent();

};




#endif 