#include "astar_utils.hh"
#include "astar_manager.hh"
#include "astar_canvas.hh"

const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#ifndef IS_TRUE
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }
#endif


int main(int argc, char* argv[])
{
    sim_params sp;

    sp.num_agents = 25;
    sp.periodic = false;
    sp.diags = true;
    sp.r_upper = 8;
    sp.diags_take_longer = true;

    sp.cells_per_side = 20;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 1.5; // 2 * 1.25; // larger value for cps = 10

    sp.dt = .1;
    sp.time_steps = 800;

    sp.gui_speedup = sp.diags_take_longer ? 1.25 : 0.75 ; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = false;
    sp.gui_random_colors = true;

    AStarManager sim = AStarManager(sp);

    {
        std::cout << "Checking nearest periodic function..." << std::endl;
        IS_TRUE(nearest_periodic(Pose(0, 1, 0, 0), Pose(8, 3, 0, 0), 5.0) == Pose(-2,3,0,0));
        IS_TRUE(nearest_periodic(Pose(0, 1, 0, 0), Pose(1, 3, 0, 0), 5.0) == Pose(1,3,0,0));
        IS_TRUE(nearest_periodic(Pose(1, 8, 0, 0), Pose(9, 0, 0, 0), 5.0) == Pose(-1,10,0,0));
        IS_TRUE(nearest_periodic(Pose(1, 1, 0, 0), Pose(1, 4, 0, 0), 2.5) == Pose(1,-1,0,0));

        // nearest_periodic(Pose(0, 1, 0, 0), Pose(8, 3, 0, 0), sim.space->cells_per_side / 2.0).Print("nearest periodic test: ");

        std::cout << "Checking diagonal distance heuristic function..." << std::endl;
        if (sp.diags) {
            IS_TRUE(sim.planner->dist_heuristic(SiteID(0,2), SiteID(5,0)) == 1.5 * 2 + 3);
            // std::cout << "should be 6 but we get " << sim.planner->dist_heuristic(SiteID(0,2), SiteID(5,0)) << std::endl;
            IS_TRUE(sim.planner->dist_heuristic(SiteID(0,5), SiteID(3,0)) == 1.5 * 3 + 2);
        }
        else {
            IS_TRUE(sim.planner->dist_heuristic(SiteID(0,2), SiteID(5,0)) == 7);
        }
        
    }


    {
        // use a single agent plan to test functions
        sp.num_agents = 1;
        sp.sensing_angle = M_PI * 2.0 / 3.0;
        sp.sensing_range = 2;
        sp.cells_per_side = 10;
        AStarManager test_sim = AStarManager(sp);

        
        test_sim.agents[0]->set_pos(SiteID(3, 0));
        test_sim.agents[0]->goal = SiteID(3, 9);
        test_sim.update();

        // for (SiteID s : test_sim.agents[0]->plan) {
        //     printf("plan step %i, %i \n", s.idx, s.idy);
        // }

        // printf("is reserved? %i", static_cast<int>(test_sim.planner->reservations[9][0][9]));

        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,2), 0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,2), M_PI / 2.0, 3, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,5), normalize(M_PI / 2.0 * 3), 4, sp.sensing_range, sp.sensing_angle)); // should be blocked

        // // Iterate through the unordered_map using a range-based for loop
        // for (const auto& pair : test_sim.planner->reservations) {
        //     const AStarPlanner::Reservation& res = pair.first;
        //     int agent_id = pair.second;

        //     std::cout << "I see a reservation at (" << res.idx << ", " << res.idy << ") at time " << res.t << " by agent " << agent_id << std::endl;
        // }


        // printf("is reserved? %i \n", static_cast<int>(test_sim.planner->reserved(2.5, 3, 2)));
        // printf("is reserved? %i \n", static_cast<int>(test_sim.planner->reserved(.5, 3, 0)));




        // periodic simulation for testing periodic sensing
        sp.periodic = true;
        AStarManager p_sim = AStarManager(sp);
        p_sim.planner->reservations[AStarPlanner::Reservation(5, 0, 0)] = -1;
        IS_TRUE(p_sim.planner->sensing_cone_occupied(SiteID(0, 9), M_PI / 2.0, 5, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(!p_sim.planner->sensing_cone_occupied(SiteID(0, 9), -M_PI / 2.0, 5, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(p_sim.planner->sensing_cone_occupied(SiteID(9, 0), 0, 5, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(!p_sim.planner->sensing_cone_occupied(SiteID(9, 0), M_PI, 5, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(p_sim.planner->sensing_cone_occupied(SiteID(9, 9), M_PI / 4.0, 5, 2 * sp.sensing_range, sp.sensing_angle));
        // IS_TRUE(!p_sim.planner->sensing_cone_occupied(SiteID(9, 9), M_PI, 5, sp.sensing_range, sp.sensing_angle));



    }



    {
        // // set two agents on course to collide
        // sim.agents[0]->set_pos(SiteID(3, 0));
        // sim.agents[0]->goal = SiteID(3, sim.sp.cells_per_side - 1);
        // sim.agents[1]->set_pos(SiteID(3, sim.sp.cells_per_side - 1));
        // sim.agents[1]->goal = SiteID(3,0);


        // sim.update();
        
        // std::cout << "Testing aborting plan...\n" << std::endl;
        // sim.agents[1]->abort_plan();


        // // set two agents on course to collide in a smaller arena
        // sim.agents[0]->set_pos(SiteID(0, 0));
        // sim.agents[0]->goal = SiteID(3, 3);
        // sim.agents[1]->set_pos(SiteID(2, 0));
        // sim.agents[1]->goal = SiteID(0,2);
    }

    {
    printf("Try opening a GUI window...\n");
    // sim.reset();

    Fl_Window win(900, 900, "A* Pathfinding");
    Canvas gui = Canvas(&sim, 0,0, win.w(), win.h());

    win.resizable(&gui);

    // win.end();
    win.show();
    gui.startAnimation();
    return Fl::run();
    }
}