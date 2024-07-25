#include "astar_planner.hh"

AStarPlanner::AStarPlanner(SpaceDiscretizer *sim_space, bool diags, int time_steps, int *t) 
    : permanent_reservations(sim_space->cells_per_side, std::vector<bool>(sim_space->cells_per_side, 0)),
    reservations(time_steps, 
                   std::vector<std::vector<bool>>(sim_space->cells_per_side, 
                                                  std::vector<bool>(sim_space->cells_per_side, false)))
{
    connect_diagonals = diags;
    space = sim_space;
    total_timesteps = time_steps;
    timestep = t; // current timestep


    // // put in some dummy obstacles
    // for (int i = 0; i < 7; i++) {
    //     reservations[i][5] = 1;
    // }

}


AStarPlanner::~AStarPlanner() {}

// TODO: update to account for periodic!
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

// /// @brief 
// /// @param start 
// /// @param goal 
// /// @param fill_reservations fill in 1s to block off entries of the reservation table corresponding to this plan
// /// @return 
// std::vector<SiteID> AStarPlanner::search_2d(SiteID start, SiteID goal) {
//     Node cur; // node we are currently exploring
//     bool found_goal = false;

//     // nodes to explore, in a set sorted in ascending order of f
//     std::set<Node, decltype(cmp)> to_visit(cmp);

//     // store details about all nodes
//     std::vector<std::vector<Node>> node_details(space->cells_per_side, std::vector<Node>(space->cells_per_side, Node()));
//     for (int i = 0; i < space->cells_per_side; i++) {
//         for (int j = 0; j < space->cells_per_side; j++) {
//             node_details[i][j].f = __FLT_MAX__;
//             node_details[i][j].g = __FLT_MAX__;
//             node_details[i][j].pos = SiteID(i, j);
//             node_details[i][j].parent = SiteID(-1, -1);
//         }
//     }

//     // initialize details about starting node
//     node_details[start.idx][start.idy] = Node(start, SiteID(-1, -1), 0, 0, 0);
//     to_visit.insert(node_details[start.idx][start.idy]);

//     // handle case where start or goal is blocked
//     if (permanent_reservations[start.idx][start.idy] || permanent_reservations[goal.idx][goal.idy]) {
//         printf("Start or goal location is blocked.\n");
//         return std::vector<SiteID>();
//     }

//     while (!to_visit.empty()) {
//         cur = *to_visit.begin();
//         to_visit.erase(to_visit.begin());

//         // if goal found
//         if (cur.pos == goal) {
//             // printf("Goal found in planner.\n");
//             found_goal = true;
//             break;
//         }

//         // for each neighbor of current cell
//         for (SpaceUnit *nbr : space->cells[cur.pos.idx][cur.pos.idy]->neighbors) {
//             int idx = nbr->id.idx;
//             int idy = nbr->id.idy;

//             float new_g = cur.g + dist_heuristic(cur.pos, nbr->id); // cost to get from start to nbr

//             if(permanent_reservations[idx][idy]) { continue; } // ignore this location if it is blocked
            
//             // if nbr has never been added to open list, or the current g is less than the one previously added
//             // update nbr values and add to open list
//             if (new_g < node_details[idx][idy].g) { 
//                 node_details[idx][idy].parent = cur.pos;
//                 node_details[idx][idy].g = new_g;
//                 node_details[idx][idy].f = new_g + dist_heuristic(nbr->id, goal);
//                 to_visit.insert(node_details[idx][idy]);
//             }
//         }
//     }

//     if (!found_goal) {
//         printf("Failed to find the goal when planning from (%i, %i) to (%i, %i).\n", start.idx, start.idy, goal.idx, goal.idy);
//         return std::vector<SiteID>();
//     }

//     return recover_plan(start, goal, &node_details);
// }

/// @brief 
/// @param start 
/// @param goal 
/// @return 
std::vector<SiteID> AStarPlanner::search(SiteID start, SiteID goal, meters_t sensing_range, radians_t sensing_angle) {
    Node cur; // node we are currently exploring
    bool found_goal = false;
    int goal_reached_time = -1;

    // nodes to explore, in a set sorted in ascending order of f
    std::set<Node, decltype(cmp)> to_visit(cmp);

    // store details about all nodes
    std::vector<std::vector<std::vector<Node>>> node_details(total_timesteps, 
                                                    std::vector<std::vector<Node>>(space->cells_per_side, 
                                                    std::vector<Node>(space->cells_per_side)));
    for (int t = 0; t < total_timesteps; t++) {
        for (int i = 0; i < space->cells_per_side; i++) {
            for (int j = 0; j < space->cells_per_side; j++) {
                node_details[t][i][j].f = __FLT_MAX__;
                node_details[t][i][j].g = __FLT_MAX__;
                node_details[t][i][j].pos = SiteID(i, j);
                node_details[t][i][j].parent = SiteID(-1, -1);
                node_details[t][i][j].t = t;
            }
        }
    }

    // initialize details about starting node
    node_details[*timestep][start.idx][start.idy] = Node(start, SiteID(-1, -1), *timestep, 0, 0);
    to_visit.insert(node_details[*timestep][start.idx][start.idy]);

    // handle case where start or goal is blocked
    if (permanent_reservations[start.idx][start.idy] || permanent_reservations[goal.idx][goal.idy]) {
        printf("Start or goal location is blocked by an obstacle.\n");
        return std::vector<SiteID>();
    }

    while (!to_visit.empty()) {
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
            float new_g = cur.g + dist_heuristic(cur.pos, nbr->id); // cost to get from start to nbr

            // Pose cur_pose = space->get_pos_as_pose(cur.pos);
            // cur_pose.a = (nbr->id - cur.pos).angle();
            
            // bool would_be_sensed = in_vision_cone(cur_pose, 
            //                                         space->get_pos_as_pose(nbr->id), 
            //                                         sensing_range, 
            //                                         sensing_angle).in_cone;

            // printf("Pos %i, %i sensed from Pos %i, %i? %i \n", idx, idy, cur.pos.idx, cur.pos.idy, would_be_sensed);
            // printf("size of to_visit: %zu \n", size(to_visit));
            bool blocked = sensing_cone_occupied(cur.pos, (nbr->id - cur.pos).angle(), cur.t, sensing_range, sensing_angle);

            if(reservations[cur.t + 1][idx][idy] // ignore this location if it is blocked
                || blocked) // or if it lies in the agent's sensing cone if they were headed this way
                { continue; } 

            // if nbr has never been added to open list, or the current g is less than the one previously added
            // update nbr values and add to open list
            if (new_g < node_details[cur.t + 1][idx][idy].g) { 
                node_details[cur.t + 1][idx][idy].parent = cur.pos;
                node_details[cur.t + 1][idx][idy].g = new_g;
                node_details[cur.t + 1][idx][idy].f = new_g + dist_heuristic(nbr->id, goal);
                to_visit.insert(node_details[cur.t + 1][idx][idy]);
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
std::vector<SiteID> AStarPlanner::recover_plan(SiteID start, SiteID goal, std::vector<std::vector<std::vector<Node>>> *node_details, int goal_reached_time) {
    std::vector<SiteID> plan;
    int time = goal_reached_time;
    SiteID s = goal;
    while (s != start) {
        reservations[time][s.idx][s.idy] = 1; // make reservation
        // printf("Reservation: time %i, pos %i, %i \n", time, s.idx, s.idy);
        SiteID step = s - (*node_details)[time][s.idx][s.idy].parent;
        plan.push_back(step);
        s = (*node_details)[time][s.idx][s.idy].parent;
        time--;
    }

    return plan;
}


std::vector<SiteID> AStarPlanner::recover_plan_2d(SiteID start, SiteID goal, std::vector<std::vector<Node>> *node_details) {
    std::vector<SiteID> plan;

    SiteID s = goal;
    while (s != start) {
        SiteID step = s - (*node_details)[s.idx][s.idy].parent;
        plan.push_back(step);
        s = (*node_details)[s.idx][s.idy].parent;
    }

    return plan;
}


// Set elements of reservation table to 0
void AStarPlanner::clear_reservations() {
    for(auto& row : permanent_reservations) {
        std::fill(row.begin(), row.end(), 0);
    }


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
                if (reservations[cur.t][test->id.idx][test->id.idy]) {
                    printf("Blocked! Sensing from %i, %i at angle %f. Test pos %i, %i. Time %i \n", sensing_from.idx, sensing_from.idy, a, test->id.idx, test->id.idy, cur.t); // debugging tomorrow: when are we blocked or not blocked ?? 
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