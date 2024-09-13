#include "astar_agent.hh"
#include "astar_planner.hh"

AStarAgent::AStarAgent(int agent_id, sim_params *sim_params, SpaceDiscretizer *sim_space, AStarPlanner *sim_planner) {
    id = agent_id;
    sp = sim_params;
    space = sim_space;
    planner = sim_planner;

    goals_reached = 0;
    goal_birth_time = *(planner->timestep);


    set_pos(random_pos());
    while (planner->reserved(*(planner->timestep), cur_pos.idx, cur_pos.idy)) {
        set_pos(random_pos());
    }
    planner->make_reservation(*(planner->timestep), cur_pos.idx, cur_pos.idy, id);

    goal = random_pos();

    if (sp->gui_random_colors) {
        color = Color::RandomColor();
    }

    else { color =  Color(0.5, 0.5, 0.5, 0.8); }
}


AStarAgent::~AStarAgent() {}

// return current position
SiteID AStarAgent::get_pos() {
    return cur_pos;
}

// if no SiteID is specified, use agent's current SiteID
Pose AStarAgent::get_pos_as_pose() {
    return space->get_pos_as_pose(cur_pos);
}



// draw
void AStarAgent::draw() {
    glPushMatrix(); // enter local agent coordinates

    Pose p = get_pos_as_pose();

    // determine agent angle
    for (std::vector<SiteID>::reverse_iterator i = plan.rbegin(); i != plan.rend(); ++i) {
        if((*i).idx != 0 || (*i).idy != 0) {
            p.a = (*i).angle();
            break;
        }
    }

    pose_shift(p);

        // draw disk at robot position
        // glColor4f(.5, .5, .5, .8); // gray
        glColor4f(color.r, color.g, color.b, 0.8);
        GLUquadric *robot_pos = gluNewQuadric();
        gluQuadricDrawStyle(robot_pos, GLU_FILL);
        gluDisk(robot_pos, 0, 0.3, 20, 1);
        gluDeleteQuadric(robot_pos);


        if (!plan.empty()) {
            // draw wedge for robot FOV
            glColor4f(0, 0, 1, 0.15); // blue
            GLUquadric *fov = gluNewQuadric();
            gluQuadricDrawStyle(fov, GLU_FILL);
            gluPartialDisk(fov, 0, sp->sensing_range, 20, 1,
                        rtod(M_PI / 2.0 + sp->sensing_angle / 2.0), // start angle
                        rtod(-sp->sensing_angle)); // sweep angle
            gluDeleteQuadric(fov);
        }
    glPopMatrix();

    // draw trail
    if(sp->gui_draw_footprints) {
        for (SiteID site_id : trail) {
            Pose p = space->get_pos_as_pose(site_id);
            glPushMatrix();
            pose_shift(p);
                // draw disk at footprint position
                // glColor4f(.5, .5, .5, .3); // gray
                glColor4f(color.r, color.g, color.b, 0.3);
                GLUquadric *robot_pos = gluNewQuadric();
                gluQuadricDrawStyle(robot_pos, GLU_FILL);
                gluDisk(robot_pos, 0, 0.3, 20, 1);
                gluDeleteQuadric(robot_pos);
            glPopMatrix();
        }
    }

    // draw small point at robot goal
    glPushMatrix(); 
        pose_shift(space->get_pos_as_pose(goal));
            // glColor4f(1, 0, .8, .7); // magenta
            glColor4f(color.r, color.g, color.b, 0.7);
            GLUquadric *goal = gluNewQuadric();
            gluQuadricDrawStyle(goal, GLU_FILL);
            gluDisk(goal, 0, 0.12, 20, 1);
            gluDeleteQuadric(goal);
    glPopMatrix();


    
}

SiteID AStarAgent::random_pos() {
    return SiteID::random(sp->cells_per_side - 1, sp->cells_per_side - 1);
}

void AStarAgent::set_pos(SiteID pos) {
    cur_pos = pos;
}

void AStarAgent::update_plan() {

    if (!planner->reserved(*(planner->timestep), cur_pos.idx, cur_pos.idy) || planner->reservations[AStarPlanner::Reservation(*(planner->timestep), cur_pos.idx, cur_pos.idy)] != id) {
        printf("\033[31mAgent %i at time %f, pos %i, %i without a reservation. \n\033[0m", id, *(planner->timestep), cur_pos.idx, cur_pos.idy);
    }
    // else {
    //     printf("Agent %i at time %f, pos %i, %i matches reservation. \n", id, *(planner->timestep), cur_pos.idx, cur_pos.idy);
    // }

    if (cur_pos == goal) {
        goal_reached_update();
    }

    if (plan.empty()) {
        get_plan();
    }

}

void AStarAgent::update_motion() {
    // Plan might be empty if goal is unreachable or if start and goal are the same location
    if (!plan.empty()) {
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

void AStarAgent::goal_reached_update() {
    while (cur_pos == goal) {
        goal = random_pos(); // new goal
    }
    goals_reached++;
    goal_birth_time = *(planner->timestep);
}

void AStarAgent::reset() {
    trail.clear();
    plan.clear();
    set_pos(random_pos());
    while (planner->reserved(*(planner->timestep), cur_pos.idx, cur_pos.idy)) {
        set_pos(random_pos());
    }
    planner->make_reservation(*(planner->timestep), cur_pos.idx, cur_pos.idy, id);
    goal = random_pos();
    goals_reached = 0;
    goal_birth_time = *(planner->timestep);

}

void AStarAgent::get_plan() {
    // printf("\nGetting a new plan for agent %i\n", id);
    plan = planner->search(cur_pos, goal, sp->sensing_range, sp->sensing_angle, id);

    // printf("Agent %i's plan: \n", id);
    // for (std::vector<SiteID>::reverse_iterator dp = plan.rbegin(); dp != plan.rend(); ++dp) {
    //     printf("step %i, %i\n", (*dp).idx, (*dp).idy);
    // }

}


void AStarAgent::abort_plan() {
    if (cur_pos == goal) {
        printf("No need to abort plan for Agent %i - goal reached!\n", id);
        goal_reached_update();
        get_plan();
        return;
    }

    if (planner->verbose) {
        printf("Aborting plan for Agent %i: \n", id);
        for (std::vector<SiteID>::reverse_iterator dp = plan.rbegin(); dp != plan.rend(); ++dp) {
            printf("step %i, %i\n", (*dp).idx, (*dp).idy);
        }
    }
    // clear existing reservations
    float dt = planner->diags_take_longer ? 0.5 : 1.0; 
    float time = *(planner->timestep);
    SiteID loc = cur_pos;

    // next step (iterate backward through plan)
    for (std::vector<SiteID>::reverse_iterator dp = plan.rbegin(); dp != plan.rend(); ++dp) {
        loc = loc + *(dp);
        time += dt;
        if (planner->reservations[AStarPlanner::Reservation(time, loc.idx, loc.idy)] != id) {
            printf("\033[31mTrying to erase a reservation (time %f, pos %i, %i) that was never made...\033[0m\n", time, loc.idx, loc.idy);
            printf("Agent %i currently at time %f, pos %i, %i\n", id, *(planner->timestep), cur_pos.idx, cur_pos.idy);
        }
        else {
            if (planner->verbose) {
                printf("Erasing reservation: time %f, pos %i, %i \n", time, loc.idx, loc.idy); // print information
            }
            planner->reservations.erase(AStarPlanner::Reservation(time, loc.idx, loc.idy));
        }
    } 

    // clear agent's plan
    plan.clear();

}