#include "astar_planner.hh"
#include "astar_agent.hh"

AStarPlanner::AStarPlanner(SpaceDiscretizer *sim_space, bool slower_diags, int time_steps, float *t, bool v) 
    : permanent_reservations(sim_space->cells_per_side, std::vector<bool>(sim_space->cells_per_side, 0))
{
    space = sim_space;
    connect_diagonals = space->connect_diagonals;
    diags_take_longer = slower_diags;
    total_timesteps = time_steps;
    timestep = t; // current timestep
    verbose = v;
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
std::vector<SiteID> AStarPlanner::search(SiteID start, SiteID goal, meters_t sensing_range, radians_t sensing_angle, int agent_id) {
    int search_rounds = 0; 

    if (verbose) {
        printf("\nAgent %i looking for plan from start %i, %i to goal %i, %i. Current time %f...\n", agent_id, start.idx, start.idy, goal.idx, goal.idy, *timestep);
    }

    Node cur; // node we are currently exploring
    bool found_goal = false;
    float goal_reached_time;

    // nodes to explore, in a set sorted in ascending order of f
    std::set<Node, decltype(cmp)> to_visit(cmp);

    // nodes already visited
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
        // // for debugging
        // printf("\n In open list...\n");
        // for (Node n : to_visit) {
        //     printf("%i, %i at time %f has f = %f and parent %i, %i\n", n.pos.idx, n.pos.idy, n.t, n.f, n.parent.idx, n.parent.idy);
        // }

        // if (search_rounds < 10) {
        //     printf("\nTop 5 elements of open list (%zu items total)...\n", size(to_visit));
        //     // Printing the first 5 elements
        //     int count = 0;
        //     for (auto it = to_visit.begin(); it != to_visit.end() && count < 5; ++it, ++count) {
        //         // std::cout << *it << std::endl;
        //         Node n = *it;
        //         printf("%i, %i at time %f has f = %f and parent %i, %i\n", n.pos.idx, n.pos.idy, n.t, n.f, n.parent.idx, n.parent.idy);
        //     }
        // }

        search_rounds++;

        cur = *to_visit.begin();
        to_visit.erase(to_visit.begin());

        // if goal found
        if (cur.pos == goal) {
            found_goal = true;
            goal_reached_time = cur.t;

            // // if (node_details.find(Reservation(cur.t, cur.pos.idx, cur.pos.idy)) == node_details.end()) { // where is this res does not print 
            //     printf("where is this reservation??\n");
            // }

            // if (!(Reservation(goal_reached_time, goal.idx, goal.idy) == Reservation(cur.t, cur.pos.idx, cur.pos.idy))) {
            //     printf("why are these different?! grt: %f, cur.t %f\n", goal_reached_time, cur.t);
            // }
        
            break;
        }
        

        // if time is out
        if (cur.t == total_timesteps) { continue; }

        // for each possible next cell to move to (including waiting in current pos)
        for (SpaceUnit *nbr : space->cells[cur.pos.idx][cur.pos.idy]->neighbors_and_me) {
            
            int idx = nbr->id.idx;
            int idy = nbr->id.idy;
            float travel_time =  diags_take_longer ? dist_heuristic(cur.pos, nbr->id) : 1.0; // set to 1 to make all steps take equal time again
            float new_g = cur.g + dist_heuristic(cur.pos, nbr->id); // cost to get from start to nbr

            // printf("nbr %i, %i:\n", idx, idy);
            bool invalid = is_invalid_step(cur.pos, nbr->id, cur.t, sensing_range, sensing_angle);
            if (reserved(cur.t + travel_time, idx, idy) // ignore this location if it is blocked
                || invalid) // or if it lies in the agent's sensing cone if they were headed this way
            {   
                // if (reserved(cur.t + travel_time, idx, idy)) {
                //     printf("time %f, nbr %i, %i is reserved by agent %i\n", cur.t + travel_time, idx, idy, reservations[Reservation(cur.t + travel_time, idx, idy)]);
                // }
                // else {
                //     printf("time %f, nbr %i, %i is probably blocked\n", cur.t, idx, idy);
                // }

                // if (!invalid & reserved(cur.t + travel_time, idx, idy)) {
                //     printf("\033[31mConfirming these are different things!!\033[0m\n");
                // }

                continue; 
            } 

            // if nbr has never been added to open list, or the current g is less than the one previously added
            // update nbr values and add to open list
            if (node_details.find(Reservation(cur.t + travel_time, idx, idy)) == node_details.end() 
                || new_g < node_details[Reservation(cur.t + travel_time, idx, idy)].g) { 
                node_details[Reservation(cur.t + travel_time, idx, idy)] = Node(SiteID(idx, idy), 
                                                                                cur.pos, 
                                                                                cur.t + travel_time, 
                                                                                new_g + dist_heuristic(nbr->id, goal), 
                                                                                new_g);
                to_visit.insert(node_details[Reservation(cur.t + travel_time, idx, idy)]);

                if (goal == SiteID(idx, idy)) {
                    if (node_details.find(Reservation(cur.t + travel_time, idx, idy)) == node_details.end()) {
                        printf("\033[31mreservation missing here too!!\033[0m\n");
                    }
                }
            }
        }
    }

    if (!found_goal) {
        printf("\033[31mFailed to find the goal when planning from (%i, %i) to (%i, %i) after %i search rounds.\n\033[0m", start.idx, start.idy, goal.idx, goal.idy, search_rounds);
        // call a replan: have this robot wait. find the robot that had this spot reserved next and make them replan. 
        // check an additional intermediate step if diags take longer
        std::vector<float> time_incs = diags_take_longer ? std::vector<float>{0.5, 1.0} : std::vector<float>{1.0};
        std::vector<SiteID> plan;
        
        // if (reserved(*timestep + 0.5, start.idx, start.idy) || reserved(*timestep + 1, start.idx, start.idy)) { // if no one is here already, have the robot wait and make whoever's coming next replan
        for (float dt : time_incs) {
            if (reserved(*timestep + dt, start.idx, start.idy)) {
                int blocker_id = reservations[Reservation(*timestep + dt, start.idx, start.idy)];
                if (blocker_id != agent_id) {
                    AStarAgent *blocker = (*agents)[blocker_id];
                    blocker->abort_plan(); // clear blocker's reservations
                    printf("Reserving a wait step for agent %i...\n", agent_id);
                    make_reservation(*timestep + dt, start.idx, start.idy, agent_id); // make wait reservation for current agent
                    plan.push_back(SiteID(0, 0));
                    printf("\033[31mCalling a replan for agent %i \n\033[0m", blocker_id);
                    blocker->get_plan(); // get new plan for blocker
                }
                else {
                    printf("blocker and agent id %i match at time %f, so no wait step was reserved...\n", agent_id, *timestep + dt);
                }
            }
            else {
                printf("Reserving a wait step for agent %i...\n", agent_id);
                make_reservation(*timestep + dt, start.idx, start.idy, agent_id); // make wait reservation for current agent
                plan.push_back(SiteID(0, 0));
            }
        }
        // else { 
        //     printf("\033[31mPlan failed but agent should have been able to wait \n\033[0m"); 
        //     printf("Reserved by agent for current step? id:  %i \n", reservations[Reservation(*timestep, start.idx, start.idy)]);
        //     printf("Reserved during next step? %i \n", reserved(*timestep + 1, start.idx, start.idy));

        //     // printf("Reserving a wait step for agent %i...\n", agent_id); // possible agent could wait but still never make it, i guess

        //     // plan.push_back(SiteID(0, 0));
        //     // make_reservation(*timestep + 1, start.idx, start.idy, agent_id); // make wait reservation for current agent
        // }

        // // Pause execution
        // std::this_thread::sleep_for(std::chrono::seconds(2));
        return plan;
    }

    else {
        // Node from_closed_list = node_details[Reservation(goal_reached_time, goal.idx, goal.idy)];
        // printf("Goal pos %i, %i. Goal f: %f\n", from_closed_list.pos.idx, from_closed_list.pos.idy, from_closed_list.f);
        // SiteID goal_parent = node_details[Reservation(goal_reached_time, goal.idx, goal.idy)].parent;
        // printf("Parent of goal %i, %i is %i, %i...", goal.idx, goal.idy, goal_parent.idx, goal_parent.idy);
        if (node_details.find(Reservation(goal_reached_time, goal.idx, goal.idy)) == node_details.end()) {
            printf("\033[31mexpected reservation for reaching goal not found!\n\033[0m");
        }

        return recover_plan(start, goal, &node_details, goal_reached_time, agent_id);
    }
}


// check if a step is valid
// cur_t is the time we arrived at SiteID cur
// nbr is the location we are considering moving to next
bool AStarPlanner::is_invalid_step(SiteID cur, SiteID nbr, float cur_t, meters_t sensing_range, radians_t sensing_angle) {
        float travel_time =  diags_take_longer ? dist_heuristic(cur, nbr) : 1.0;

        // wrap sensing angle if needed
        SiteID wrapped;
        if (space->periodic) {
            Pose pose_wrapped = nearest_periodic(Pose(cur.idx, cur.idy, 0, 0), Pose(nbr.idx, nbr.idy, 0, 0), space->cells_per_side / 2.0);
            wrapped = SiteID(pose_wrapped.x, pose_wrapped.y);
        }
        else { wrapped = nbr; }

        // first, check that the positions we will occupy are not already reserved
        bool positions_invalid;
        positions_invalid = reserved(cur_t + travel_time, nbr.idx, nbr.idy);

        if (!positions_invalid && diags_take_longer) { // extra intermediate positions to check
            if (abs(nbr.idx - cur.idx) + abs(nbr.idy - cur.idy) > 1) { // check if step is diagonal
                positions_invalid = reserved(cur_t + 0.5, cur.idx, cur.idy) || reserved(cur_t + 1.0, cur.idx, cur.idy);
            }
            else {
                positions_invalid = reserved(cur_t + 0.5, cur.idx, cur.idy);
            }
        }

        if (positions_invalid) {
            // printf("nbr %i, %i is invalid due to another agent's reservation\n", nbr.idx, nbr.idy);
            return true;
        }


        bool sensing_cone_blocked;
        // now check sensing cone validity
        // if moving to this neighbor would require moving while something was in our sensing cone, this path is blocked. 
        if (wrapped == cur) { sensing_cone_blocked = false; } // sensing cone never blocked if this is a wait step and we've already checked that 

        // check that sensing cone is not blocked the step right before moving
        // case where time increments by 1 per step
        else if (!diags_take_longer) { sensing_cone_blocked = sensing_cone_occupied(cur, (wrapped - cur).angle(), cur_t, sensing_range, sensing_angle); }

        // handling the case where diagonals take longer and time increments by 0.5 per step
        else {  sensing_cone_blocked = sensing_cone_occupied(cur, (wrapped - cur).angle(), cur_t + travel_time - 0.5, sensing_range, sensing_angle);  }


        // if (sensing_cone_blocked) {
        //     printf("nbr %i, %i is invalid because sensing cone would be blocked while traveling there\n", nbr.idx, nbr.idy);
        // }

        return sensing_cone_blocked;
 }


// Extract plan back out from table of graph search data
// Also makes reservations in the reservation table
std::vector<SiteID> AStarPlanner::recover_plan(SiteID start, SiteID goal, std::unordered_map<Reservation, Node, Reservation::hash> *node_details, float goal_reached_time, int agent_id) {
    std::vector<SiteID> plan;
    float time = goal_reached_time;
    SiteID s = goal; // current location we are tracing

    while (s != start || time != *timestep) {
        // printf("here at %i, %i at time %f\n", s.idx, s.idy, time);

        make_reservation(time, s.idx, s.idy, agent_id);
        SiteID step = s - (*node_details)[Reservation(time, s.idx, s.idy)].parent;
        // printf("previous step: %i, %i \n", step.idx, step.idy);

        if ((*node_details).find(Reservation(time, s.idx, s.idy)) == (*node_details).end()) {
            printf("\033[31mThis reservation has never been made. \n\033[0m");
        }

        if (diags_take_longer) {
            s = (*node_details)[Reservation(time, s.idx, s.idy)].parent;
            
            if (abs(step.idx) + abs(step.idy) <= 1) { // if not a diagonal, this motion takes two half-timesteps
                plan.push_back(step);
                make_reservation(time - 0.5, s.idx, s.idy, agent_id);
                plan.push_back(SiteID(0,0));
                time = time - 1;
            }
            else { // if a diagonal, it takes three half-timesteps
                plan.push_back(step);
                make_reservation(time - 0.5, s.idx, s.idy, agent_id);
                plan.push_back(SiteID(0,0));
                make_reservation(time - 1.0, s.idx, s.idy, agent_id);
                plan.push_back(SiteID(0,0));
                time = time - 1.5;
            }
        }
        
        // for when diagonals take the same time as adjacents
        else {
            plan.push_back(step);
            s = (*node_details)[Reservation(time, s.idx, s.idy)].parent;
            time--;
        }

        if (time < 0) {
            printf("\033[31merror: t < 0 while tracing plan \033[0m\n");
            break;
        }
    }

    if (time != *timestep) {
        printf("\033[31mPlan start time does not match first reservation \033[0m\n");
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

bool AStarPlanner::sensing_cone_occupied(SiteID sensing_from, radians_t a, float t, meters_t sensing_range, radians_t sensing_angle) {
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
                    // printf("Blocked! Sensing from %i, %i at angle %f. Test pos %i, %i. Time %f \n", sensing_from.idx, sensing_from.idy, a, test->id.idx, test->id.idy, cur.t); // debugging tomorrow: when are we blocked or not blocked ?? 
                    return true; // sensing cone is blocked
                }

                if (visited.find(test->id) == visited.end()) {
                    meters_t dist = p.Distance(test_pose);
                    to_visit.insert(Node(test->id, SiteID(-1, -1), cur.t, dist, dist));
                }
            }
        }
    }

    return false;
}