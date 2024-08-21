#include "astar_agent.hh"
#include "astar_planner.hh"

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

void AStarAgent::update() {
    while (cur_pos == goal) {
        goal = random_pos(); // new goal
        // printf("plan size left after goal: %zu \n", size(plan));
    }

    if (plan.empty()) {
        get_plan();
    }

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

void AStarAgent::reset() {
    trail.clear();
    plan.clear();
    set_pos(random_pos());
    goal = random_pos();

}

void AStarAgent::get_plan() {
    printf("\nGetting a new plan for agent %i\n", id);
    plan = planner->search(cur_pos, goal, sp->sensing_range, sp->sensing_angle, id);
}


void AStarAgent::abort_plan() {
    //
    printf("plan to abort: \n");
    for (std::vector<SiteID>::reverse_iterator dp = plan.rbegin(); dp != plan.rend(); ++dp) {
        printf("step %i, %i\n", (*dp).idx, (*dp).idy);
    }

    // clear existing reservations
    float dt = planner->diags_take_longer ? 0.5 : 1.0; 
    float time = *(planner->timestep);
    SiteID loc = cur_pos;

    // next step (iterate backward through plan)
    for (std::vector<SiteID>::reverse_iterator dp = plan.rbegin(); dp != plan.rend(); ++dp) {
        if (!planner->reserved(time, loc.idx, loc.idy)) {
            printf("\033[31mTrying to erase a reservation (time %f, pos %i, %i) that was never made...\033[0m\n", time, loc.idx, loc.idy);
        }
        else {
            printf("Erasing reservation: time %f, pos %i, %i \n", time, loc.idx, loc.idy); // print information
            planner->reservations.erase(AStarPlanner::Reservation(time, loc.idx, loc.idy));

            loc = loc + *(dp);
            time += dt;
        }
    } 

    // clear agent's plan
    plan.clear();

}