#include "astar_planner.hh"

AStarPlanner::AStarPlanner(SpaceDiscretizer *sim_space, bool diags, int time_steps, int *t) 
    : permanent_reservations(sim_space->cells_per_side, std::vector<bool>(sim_space->cells_per_side, 0))
{
    connect_diagonals = diags;
    space = sim_space;
    total_timesteps = time_steps;
    timestep = t; // current timestep
}


AStarPlanner::~AStarPlanner() {}

// distance heuristic used in a* search
float AStarPlanner::dist_heuristic(SiteID a, SiteID b) {
    // waiting can't be free!
    if (a == b) {
        return 1;
    }

    if (space->periodic) {
        Pose pose_b = nearest_periodic(Pose(a.idx, a.idy, 0, 0), Pose(b.idx, b.idy, 0, 0), space->cells_per_side / 2.0);
        b = SiteID(pose_b.x, pose_b.y);
    }

    if (connect_diagonals) { // distance allowing diagonal motion
        int dx = abs(a.idx - b.idx);
        int dy = abs(a.idy - b.idy);
        // using 1.5 as the distance of a diagonal step (true distance is sqrt2 but we want a rational number for our time discretization later on)
        return (dx + dy) - 2 * fmin(dx, dy) + fmin(dx, dy) * 1.5;
    }

    else { return abs(a.idx - b.idx) + abs(a.idy - b.idy); } // Manhattan distance
}


// comparison function for ordering nodes
auto cmp = [](AStarPlanner::Node a, AStarPlanner::Node b) { 
    if (a.f != b.f) { return a.f < b.f; }
    else if (a.t != b.t) { return a.t > b.t; }
    else if (a.pos.idx != b.pos.idx) { return a.pos.idx < b.pos.idx; }
    else { return a.pos.idy < b.pos.idy; }
};


// uses a* search to return a set of movements to go from start to goal
// accounts for spacetime reservations made by other agents, and for agent sensing cone
std::vector<SiteID> AStarPlanner::search(SiteID start, SiteID goal, meters_t sensing_range, radians_t sensing_angle) {
    Node cur; // node we are currently exploring
    bool found_goal = false;
    int goal_reached_time = -1;

    // nodes to explore, in a set sorted in ascending order of f
    std::set<Node, decltype(cmp)> to_visit(cmp);

    // // store details about all nodes
    // std::vector<std::vector<std::vector<Node>>> node_details(total_timesteps, 
    //                                                 std::vector<std::vector<Node>>(space->cells_per_side, 
    //                                                 std::vector<Node>(space->cells_per_side)));
    // for (int t = 0; t < total_timesteps; t++) {
    //     for (int i = 0; i < space->cells_per_side; i++) {
    //         for (int j = 0; j < space->cells_per_side; j++) {
    //             node_details[t][i][j].f = __FLT_MAX__;
    //             node_details[t][i][j].g = __FLT_MAX__;
    //             node_details[t][i][j].pos = SiteID(i, j);
    //             node_details[t][i][j].parent = SiteID(-1, -1);
    //             node_details[t][i][j].t = t;
    //         }
    //     }
    // }

    // // initialize details about starting node
    // node_details[*timestep][start.idx][start.idy] = Node(start, SiteID(-1, -1), *timestep, 0, 0);
    // to_visit.insert(node_details[*timestep][start.idx][start.idy]);

    std::unordered_map<Reservation, Node, Reservation::hash> node_details;   
    // initialize details about starting node
    node_details[Reservation(*timestep, start.idx, start.idy)] = Node(start, SiteID(-1, -1), *timestep, 0, 0);
    to_visit.insert(node_details[Reservation(*timestep, start.idx, start.idy)]);


    // // handle case where start or goal is blocked
    // if (permanent_reservations[start.idx][start.idy] || permanent_reservations[goal.idx][goal.idy]) {
    //     printf("Start or goal location is blocked by an obstacle.\n");
    //     return std::vector<SiteID>();
    // }

    while (!to_visit.empty()) {
        // for debugging
        printf("\n In open list...\n");
        for (Node n : to_visit) {
            printf("%i, %i at time %f has f = %f \n", n.pos.idx, n.pos.idy, n.t, n.f);
        }


        cur = *to_visit.begin();
        to_visit.erase(to_visit.begin());

        // if goal found
        if (cur.pos == goal) {
            // printf("Goal found in planner.\n");
            found_goal = true;
            goal_reached_time = cur.t;
            break;
        }

        // if time is out
        if (cur.t == total_timesteps) { continue; }

        // for each possible next cell to move to (including waiting in current pos)
        for (SpaceUnit *nbr : space->cells[cur.pos.idx][cur.pos.idy]->neighbors_and_me) {
            
            int idx = nbr->id.idx;
            int idy = nbr->id.idy;
            float travel_time = 1; // dist_heuristic(cur.pos, nbr->id);
            float new_g = cur.g + dist_heuristic(cur.pos, nbr->id); //travel_time; // cost to get from start to nbr

            // wrap sensing angle if needed
            SiteID wrapped;
            if (space->periodic) {
                Pose pose_wrapped = nearest_periodic(Pose(cur.pos.idx, cur.pos.idy, 0, 0), Pose(nbr->id.idx, nbr->id.idy, 0, 0), space->cells_per_side / 2.0);
                wrapped = SiteID(pose_wrapped.x, pose_wrapped.y);
            }
            else { wrapped = nbr->id; }

            // try to implement different times for diagonal steps
            bool blocked = sensing_cone_occupied(cur.pos, (wrapped - cur.pos).angle(), cur.t, sensing_range, sensing_angle);

            if(reserved(cur.t + travel_time, idx, idy) // ignore this location if it is blocked
                || blocked) // or if it lies in the agent's sensing cone if they were headed this way
                { continue; } 

            // bool blocked = sensing_cone_occupied(cur.pos, (wrapped - cur.pos).angle(), cur.t, sensing_range, sensing_angle);

            // if(reserved(cur.t + 1, idx, idy) // ignore this location if it is blocked
            //     || blocked) // or if it lies in the agent's sensing cone if they were headed this way
            //     { continue; } 

            // if nbr has never been added to open list, or the current g is less than the one previously added
            // update nbr values and add to open list


            // if (new_g < node_details[cur.t + 1][idx][idy].g) { 
            //     node_details[cur.t + 1][idx][idy].parent = cur.pos;
            //     node_details[cur.t + 1][idx][idy].g = new_g;
            //     node_details[cur.t + 1][idx][idy].f = new_g + dist_heuristic(nbr->id, goal);
            //     to_visit.insert(node_details[cur.t + 1][idx][idy]);
            // }


            if (node_details.find(Reservation(cur.t + travel_time, idx, idy)) == node_details.end() || new_g < node_details[Reservation(cur.t + travel_time, idx, idy)].g) { 
                node_details[Reservation(cur.t + travel_time, idx, idy)] = Node(SiteID(idx, idy), 
                                                                                cur.pos, 
                                                                                cur.t + travel_time, 
                                                                                new_g + dist_heuristic(nbr->id, goal), 
                                                                                new_g);
                to_visit.insert(node_details[Reservation(cur.t + travel_time, idx, idy)]);
            }

        }
    }

    if (!found_goal) {
        printf("Failed to find the goal when planning from (%i, %i) to (%i, %i).\n", start.idx, start.idy, goal.idx, goal.idy);
        return std::vector<SiteID>();
    }

    return recover_plan(start, goal, &node_details, goal_reached_time);
}


// Extract plan back out from table of graph search data
// Also makes reservations in the reservation table
std::vector<SiteID> AStarPlanner::recover_plan(SiteID start, SiteID goal, std::unordered_map<Reservation, Node, Reservation::hash> *node_details, int goal_reached_time) {
    std::vector<SiteID> plan;
    float time = goal_reached_time;
    SiteID s = goal; // current location we are tracing
    while (s != start) {
        printf("here at time %f\n", time);
        reservations.insert(Reservation(time, s.idx, s.idy)); // make reservation
        SiteID step = s - (*node_details)[Reservation(time, s.idx, s.idy)].parent;


        // s = (*node_details)[Reservation(time, s.idx, s.idy)].parent;

        // // NEED TO FILL IN MORE RESERVATIONS IN HERE
        // // rewrite for when diagonals take longer
        // if (abs(step.idx) + abs(step.idy) == 1) { // if not a diagonal, this motion takes two half-timesteps
        //     plan.push_back(step);
        //     reservations.insert(Reservation(time - 0.5, s.idx, s.idy)); // s is now the parent
        //     plan.push_back(SiteID(0,0));
        //     time = time - 1;
        // }
        // else { // if a diagonal, it takes three half-timesteps
        //     plan.push_back(step);
        //     reservations.insert(Reservation(time - 0.5, s.idx, s.idy));
        //     plan.push_back(SiteID(0,0));
        //     reservations.insert(Reservation(time - 1.0, s.idx, s.idy));
        //     plan.push_back(SiteID(0,0));
        //     time = time - 1.5;
        // }
        
        // printf("Reservation: time %i, pos %i, %i \n", time, s.idx, s.idy);
        plan.push_back(step);
        s = (*node_details)[Reservation(time, s.idx, s.idy)].parent;
        time--;

        if (time < 0) {
            printf("error: time < 0 while tracing plan\n");
            break;
        }
    }

    return plan;
}

// Set elements of reservation table to 0
void AStarPlanner::clear_reservations() {
    for(auto& row : permanent_reservations) {
        std::fill(row.begin(), row.end(), 0);
    }

    reservations.clear();


}   



bool AStarPlanner::sensing_cone_occupied(SiteID sensing_from, radians_t a, int t, meters_t sensing_range, radians_t sensing_angle) {
    Pose p = space->get_pos_as_pose(sensing_from);
    p.a = a;

    Node cur; // node we are currently exploring to see if it is reserved and within vision cone

    // nodes to explore, in a set sorted in ascending order of f
    std::set<Node, decltype(cmp)> to_visit(cmp);
    to_visit.insert(Node(sensing_from, SiteID(-1, -1), t, 0, 0)); // repurpose f in this function to be distance from starting node

    // track nodes we have already visited
    std::unordered_set<SiteID, SiteID::hash> visited;

    while (!to_visit.empty()) {
        cur = *to_visit.begin();
        to_visit.erase(to_visit.begin());
        visited.insert(cur.pos);

        for (SpaceUnit *test: space->cells[cur.pos.idx][cur.pos.idy]->neighbors) {
            Pose test_pose = space->get_pos_as_pose(test->id);
            if (space->periodic) { test_pose = nearest_periodic(p, test_pose, space->space_r); }

            if (in_vision_cone(p, test_pose, sensing_range, sensing_angle).in_cone) {
                // if (sensing_from == SiteID(3,0) && a == -M_PI / 2) {
                //     // draw for debugging
                //         glPushMatrix(); // enter local agent coordinates
                //             pose_shift(test_pose);
                //                 // draw disk at robot position
                //                 glColor4f(0, 1, 1, .8);
                //                 GLUquadric *robot_pos = gluNewQuadric();
                //                 gluQuadricDrawStyle(robot_pos, GLU_FILL);
                //                 gluDisk(robot_pos, 0, 0.1, 20, 1);
                //                 gluDeleteQuadric(robot_pos);
                //         glPopMatrix();
                // }

                if (reserved(cur.t, test->id.idx, test->id.idy)) {
                    printf("Blocked! Sensing from %i, %i at angle %f. Test pos %i, %i. Time %f \n", sensing_from.idx, sensing_from.idy, a, test->id.idx, test->id.idy, cur.t); // debugging tomorrow: when are we blocked or not blocked ?? 
                    return true; // sensing cone is blocked
                }

                if (visited.find(test->id) == visited.end()) {
                    // if (sp->periodic) { nbr_pos = nearest_periodic(agent_pos, nbr_pos, sp->r_upper); }
                    meters_t dist = p.Distance(test_pose);
                    to_visit.insert(Node(test->id, SiteID(-1, -1), cur.t, dist, dist));
                }
            }
        }
    }

    return false;
}