#ifndef ASTAR_AGENT
#define ASTAR_AGENT

#include "../random.hh"
#include "astar_utils.hh"
#include "../shared_utils.hh"
#include <deque>

class AStarAgent {
    public:

    int id;
    SiteID goal;
    SiteID cur_pos;
    sim_params *sp;
    SpaceDiscretizer *space;
    Color color;

    // store recent poses
    std::deque<SiteID> trail;

    // plan for upcoming steps
    std::vector<SiteID> plan;

    void get_plan();

    void update();

    void reset();

    void set_pos(SiteID pos);

    SiteID get_pos();

    Pose get_pos_as_pose();
    Pose get_pos_as_pose(SiteID site_id);

    void update_trail();

    void draw();

    SiteID random_pos();

    // Constructor
    AStarAgent(int agent_id, sim_params *sim_params, SpaceDiscretizer *sim_space);

    // Destructor
    ~AStarAgent();

};




#endif 