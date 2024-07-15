#include "astar_agent.hh"

AStarAgent::AStarAgent(int agent_id, sim_params *sim_params, SpaceDiscretizer *sim_space) {
    id = agent_id;
    sp = sim_params;
    space = sim_space;
    set_pos(random_pos());
}


AStarAgent::~AStarAgent() {}

Pose AStarAgent::get_pos_as_pose() {
    SpaceUnit *su = space->cells[cur_pos.idx][cur_pos.idy];
    return Pose(su->x, su->y, 0, 0);
}


// draw
void AStarAgent::draw() {
    glPushMatrix(); // enter local agent coordinates
    pose_shift(get_pos_as_pose());

        // draw disk at robot position
        glColor4f(.5, .5, .5, .8); // gray
        GLUquadric *robot_pos = gluNewQuadric();
        gluQuadricDrawStyle(robot_pos, GLU_FILL);
        gluDisk(robot_pos, 0, 0.15, 20, 1);
        gluDeleteQuadric(robot_pos);

    glPopMatrix();

    // // draw trail
    // if(sp->gui_draw_footprints) {
    //     for (Pose p : trail) {
    //         glPushMatrix();
    //         pose_shift(p);
    //             // draw disk at footprint position
    //             glColor4f(.5, .5, .5, .3); // gray
    //             GLUquadric *robot_pos = gluNewQuadric();
    //             gluQuadricDrawStyle(robot_pos, GLU_FILL);
    //             gluDisk(robot_pos, 0, 0.15, 20, 1);
    //             gluDeleteQuadric(robot_pos);
    //         glPopMatrix();
    //     }
    // }
    
}

SiteID AStarAgent::random_pos() {
    return SiteID::random(sp->cells_per_side - 1, sp->cells_per_side - 1);
}

void AStarAgent::set_pos(SiteID pos) {
    cur_pos = pos;
}

void AStarAgent::update() {
    if (size(plan) == 0) {
        get_plan();
    }

    SiteID dp = plan.back(); // change in position
    plan.pop_back();
    
    SiteID next_pos = cur_pos + dp;

    // if new position is out of bounds
    if (next_pos.idx < 0 || next_pos.idy < 0 || next_pos.idx >= sp->cells_per_side || next_pos.idy >= sp->cells_per_side) {
        // if space not periodic, don't move

        // else if space periodic,
        if (sp->periodic) {
            int wrap_idx = (next_pos.idx + sp->cells_per_side) % sp->cells_per_side;
            int wrap_idy = (next_pos.idy + sp->cells_per_side) % sp->cells_per_side;
            set_pos(SiteID(wrap_idx, wrap_idy));
        }
    }

    else { set_pos(next_pos); }
}

void AStarAgent::reset() {
    trail.clear();
    plan.clear();
    set_pos(random_pos());

}

void AStarAgent::get_plan() {
    // plan.clear();
    plan = std::vector<SiteID>(5, SiteID(1, 0));

}