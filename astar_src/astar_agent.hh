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

    // store recent poses
    std::deque<SiteID> trail;

    // plan for upcoming steps
    std::vector<SiteID> plan;

    void get_plan();

    void abort_plan();

    void update_plan();

    void update_motion();

    void goal_reached_update();

    void reset();

    void set_pos(SiteID pos);

    SiteID get_pos();

    Pose get_pos_as_pose();

    void update_trail();

    void draw();

    SiteID random_pos();

    // Constructor
    AStarAgent(int agent_id, sim_params *sim_params, SpaceDiscretizer *sim_space, AStarPlanner *sim_planner);

    // Destructor
    ~AStarAgent();

};




#endif 