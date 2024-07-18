#include "astar_agent.hh"

AStarAgent::AStarAgent(int agent_id, sim_params *sim_params, SpaceDiscretizer *sim_space, AStarPlanner *sim_planner) {
    id = agent_id;
    sp = sim_params;
    space = sim_space;
    planner = sim_planner;
    set_pos(random_pos());
    goal = random_pos();

    if (sp->gui_random_colors) {
        color = Color::RandomColor();
    }

    else { color =  Color(0.5, 0.5, 0.5, 0.8); }
}


AStarAgent::~AStarAgent() {}

// return current positoin
SiteID AStarAgent::get_pos() {
    return cur_pos;
}

// get coordinate position of a SiteID
Pose AStarAgent::get_pos_as_pose(SiteID site_id) {
    SpaceUnit *su = space->cells[site_id.idx][site_id.idy];
    return Pose(su->x, su->y, 0, 0);
}

// if no SiteID is specified, use agent's current SiteID
Pose AStarAgent::get_pos_as_pose() {
    return get_pos_as_pose(cur_pos);
}



// draw
void AStarAgent::draw() {
    glPushMatrix(); // enter local agent coordinates
    pose_shift(get_pos_as_pose());

        // draw disk at robot position
        // glColor4f(.5, .5, .5, .8); // gray
        glColor4f(color.r, color.g, color.b, 0.8);

        GLUquadric *robot_pos = gluNewQuadric();
        gluQuadricDrawStyle(robot_pos, GLU_FILL);
        gluDisk(robot_pos, 0, 0.4, 20, 1);
        gluDeleteQuadric(robot_pos);

    glPopMatrix();

    // draw trail
    if(sp->gui_draw_footprints) {
        for (SiteID site_id : trail) {
            Pose p = get_pos_as_pose(site_id);
            glPushMatrix();
            pose_shift(p);
                // draw disk at footprint position
                // glColor4f(.5, .5, .5, .3); // gray
                glColor4f(color.r, color.g, color.b, 0.3);
                
                GLUquadric *robot_pos = gluNewQuadric();
                gluQuadricDrawStyle(robot_pos, GLU_FILL);
                gluDisk(robot_pos, 0, 0.4, 20, 1);
                gluDeleteQuadric(robot_pos);
            glPopMatrix();
        }

    // draw small point at robot goal
    glPushMatrix(); 
        pose_shift(get_pos_as_pose(goal));
            // glColor4f(1, 0, .8, .7); // magenta
            glColor4f(color.r, color.g, color.b, 0.7);
            GLUquadric *goal = gluNewQuadric();
            gluQuadricDrawStyle(goal, GLU_FILL);
            gluDisk(goal, 0, 0.12, 20, 1);
            gluDeleteQuadric(goal);
    glPopMatrix();
    }


    
}

SiteID AStarAgent::random_pos() {
    return SiteID::random(sp->cells_per_side - 1, sp->cells_per_side - 1);
}

void AStarAgent::set_pos(SiteID pos) {
    cur_pos = pos;
}

void AStarAgent::update() {
    if (cur_pos == goal) {
        goal = random_pos(); // new goal
        printf("plan size left after goal: %zu \n", size(plan));
    }

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

    // add new position to trail
    if (sp->gui_draw_footprints) {
        update_trail();
    }
}

void AStarAgent::update_trail() {
    trail.push_back(get_pos());
    if(trail.size() > 6) {
        trail.pop_front();
    }
}

void AStarAgent::reset() {
    trail.clear();
    plan.clear();
    set_pos(random_pos());
    goal = random_pos();

}

void AStarAgent::get_plan() {
    // plan.clear();
    // std::vector<SiteID> directions = {SiteID(1,0), SiteID(0,1), SiteID(-1, 0), SiteID(0, -1)};

    // plan = std::vector<SiteID>(3, directions[Random::get_unif_int(0, 3)]);

    // plan = std::vector<SiteID>(5, SiteID(1, 0));

    printf("Planning goal to (%i, %i)... \n", goal.idx, goal.idy);
    plan = planner->search_2d(cur_pos, goal);

}