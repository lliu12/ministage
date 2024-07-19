#include "astar_planner.hh"

AStarPlanner::AStarPlanner(SpaceDiscretizer *sim_space, bool diags) 
    : reservations(sim_space->cells_per_side, std::vector<bool>(sim_space->cells_per_side, 0))
{
    connect_diagonals = diags;
    space = sim_space;


    // put in some dummy obstacles
    for (int i = 0; i < 7; i++) {
        reservations[i][5] = 1;
    }

}


AStarPlanner::~AStarPlanner() {}

// TODO: update to account for periodic!
float AStarPlanner::dist_heuristic(SiteID a, SiteID b) {
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

// datatype for storing relevant information
struct Node {
    // f = g + h
    double f, g;
    SiteID pos;
    SiteID parent;
     
    // constructor
    Node(SiteID my_pos, SiteID my_parent, double f_val, double g_val)
        : pos(my_pos), parent(my_parent), f(f_val), g(g_val)
    {}

    Node() {}
};

// comparison function for ordering nodes
auto cmp = [](Node a, Node b) { 
    if (a.f != b.f) { return a.f < b.f; }
    else if (a.pos.idx != b.pos.idx) { return a.pos.idx < b.pos.idx; }
    else { return a.pos.idy < b.pos.idy; }
};

/// @brief 
/// @param start 
/// @param goal 
/// @param fill_reservations fill in 1s to block off entries of the reservation table corresponding to this plan
/// @return 
std::vector<SiteID> AStarPlanner::search_2d(SiteID start, SiteID goal) {
    Node cur; // node we are currently exploring
    bool found_goal = false;

    // nodes to explore, in a set sorted in ascending order of f
    std::set<Node, decltype(cmp)> to_visit(cmp);

    // // nodes already explored, denoted by 1s in a lookup table
    // std::vector<std::vector<bool>> visited(space->cells_per_side, std::vector<bool>(space->cells_per_side, false));

    // store details about all nodes
    std::vector<std::vector<Node>> node_details(space->cells_per_side, std::vector<Node>(space->cells_per_side, Node()));
    for (int i = 0; i < space->cells_per_side; i++) {
        for (int j = 0; j < space->cells_per_side; j++) {
            node_details[i][j].f = __FLT_MAX__;
            node_details[i][j].g = __FLT_MAX__;
            node_details[i][j].pos = SiteID(i, j);
            node_details[i][j].parent = SiteID(-1, -1);
        }
    }

    // initialize details about starting node
    node_details[start.idx][start.idy] = Node(start, SiteID(-1, -1), 0, 0);
    to_visit.insert(node_details[start.idx][start.idy]);

    // handle case where start or goal is blocked
    if (reservations[start.idx][start.idy] || reservations[goal.idx][goal.idy]) {
        printf("Start or goal location is blocked.\n");
        return std::vector<SiteID>();
    }

    while (!to_visit.empty()) {
        cur = *to_visit.begin();
        to_visit.erase(to_visit.begin());

        // printf("Visiting node (%i, %i)... \n", cur.pos.idx, cur.pos.idy);
        // for (Node nn : to_visit) {
        //     printf("On open list: node (%i, %i) with f = %f \n", nn.pos.idx, nn.pos.idy, nn.f);
        //     // std::cout << nn.pos.idx << nn.pos.idy << nn.f << std::endl;
        //     // printf("\n");
        // }
        // printf("\n");

        // if goal found
        if (cur.pos == goal) {
            // printf("Goal found in planner.\n");
            found_goal = true;
            break;
        }

        // for each neighbor of current cell
        for (SpaceUnit *nbr : space->cells[cur.pos.idx][cur.pos.idy]->neighbors) {
            int idx = nbr->id.idx;
            int idy = nbr->id.idy;

            float new_g = cur.g + dist_heuristic(cur.pos, nbr->id); // cost to get from start to nbr

            if(reservations[idx][idy]) { continue; } // ignore this location if it is blocked

            // if(nbr->id == SiteID(2,7)) {
            //     printf("EXPLORING 2, 7\n");
            //     printf("tentative g is %f and 2,7's node details g is %f\n", new_g, node_details[idx][idy].g);
            // }
            
            // if nbr has never been added to open list, or the current g is less than the one previously added
            // update nbr values and add to open list
            if (new_g < node_details[idx][idy].g) { 
                node_details[idx][idy].parent = cur.pos;
                node_details[idx][idy].g = new_g;
                node_details[idx][idy].f = new_g + dist_heuristic(nbr->id, goal);
                // node_details[idx][idy].h = dist_heuristic(nbr->id, goal);
                to_visit.insert(node_details[idx][idy]);

                // if(nbr->id == SiteID(2,7)) {
                //     printf("SHOULD HAVE INSERTED 2,7 BUT OPEN LIST IS:\n");
                //     printf("neighbor info: %i, %i\n", node_details[idx][idy].pos.idx, node_details[idx][idy].pos.idy);
                //     for (Node nn : to_visit) {
                //         printf("On open list: node (%i, %i) with f = %f \n", nn.pos.idx, nn.pos.idy, nn.f);
                //         // std::cout << nn.pos.idx << nn.pos.idy << nn.f << std::endl;
                //         // printf("\n");
                //     }

                // printf("EXPLORING 2, 7\n");
                // printf("tentative g is %f and 2,7's node details g is %f\n", new_g, node_details[idx][idy].g);
                // }
            }



        }

        // visited[cur.pos.idx][cur.pos.idy] == true;

    }

    if (!found_goal) {
        printf("Failed to find the goal when planning from (%i, %i) to (%i, %i).\n", start.idx, start.idy, goal.idx, goal.idy);
        return std::vector<SiteID>();
    }

    // recover plan
    std::vector<SiteID> plan;

    SiteID s = goal;
    while (s != start) {
        SiteID step = s - node_details[s.idx][s.idy].parent;
        plan.push_back(step);
        s = node_details[s.idx][s.idy].parent;
    }

    return plan;
}


// Set elements of reservation table to 0
void AStarPlanner::clear_reservations() {
    for(auto& row : reservations) {
        std::fill(row.begin(), row.end(), 0);
    }
}   
