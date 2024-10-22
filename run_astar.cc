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

    sp.num_agents = 10;
    sp.periodic = true;
    sp.diags = true;
    sp.r_upper = 8;
    sp.diags_take_longer = true;

    sp.cells_per_side = 20;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 2 * 1.25; // larger value for cps = 10

    sp.dt = .1;
    sp.time_steps = 800;

    sp.gui_speedup = sp.diags_take_longer ? 1.25 : 0.75 ; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = false;
    sp.gui_random_colors = true;

    sp.verbose = true;

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
        sp.sensing_range = 2.5;
        sp.r_upper = 8;
        sp.cells_per_side = 10;
        sp.periodic = false;
        sp.diags_take_longer = false;

        AStarManager test_sim = AStarManager(sp);
        

        
        test_sim.agents[0]->set_pos(SiteID(3, 0));
        test_sim.agents[0]->goal = SiteID(3, 9);
        test_sim.planner->reservations.clear();
        test_sim.planner->make_reservation(0, 3, 0, 0);
        test_sim.update();

        // for (SiteID s : test_sim.agents[0]->plan) {
        //     printf("plan step %i, %i \n", s.idx, s.idy);
        // }

        // printf("is reserved? %i", static_cast<int>(test_sim.planner->reservations[9][0][9]));

        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,2), 0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,2), M_PI / 2.0, 3, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,5), normalize(M_PI / 2.0 * 3), 4, sp.sensing_range, sp.sensing_angle)); // should be blocked

        IS_TRUE(recover_periodic_step(SiteID(-9, 1), 10) == SiteID(1, 1));
        IS_TRUE(recover_periodic_step(SiteID(9, 1), 10) == SiteID(-1, 1));
        IS_TRUE(recover_periodic_step(SiteID(9, 9), 10) == SiteID(-1, -1));
        IS_TRUE(recover_periodic_step(SiteID(19, 1), 20) == SiteID(-1, 1));
        IS_TRUE(recover_periodic_step(SiteID(0, 0), 20) == SiteID(0, 0));



        // // Iterate through the unordered_map using a range-based for loop
        // for (const auto& pair : test_sim.planner->reservations) {
        //     const AStarPlanner::Reservation& res = pair.first;
        //     int agent_id = pair.second;

        //     std::cout << "I see a reservation at (" << res.idx << ", " << res.idy << ") at time " << res.t << " by agent " << agent_id << std::endl;
        // }


        // printf("is reserved? %i \n", static_cast<int>(test_sim.planner->reserved(2.5, 3, 2)));
        // printf("is reserved? %i \n", static_cast<int>(test_sim.planner->reserved(.5, 3, 0)));


        // test sensing_cone_invalid (blocking the neighbor also not allowed)
        // Agent 0 looking for plan from start 3, 0 to goal 3, 9. Current time 0.000000...
        // Reservation: time 9.000000, pos 3, 9 
        // Reservation: time 8.000000, pos 3, 8 
        // Reservation: time 7.000000, pos 3, 7 
        // Reservation: time 6.000000, pos 3, 6 
        // Reservation: time 5.000000, pos 3, 5 
        // Reservation: time 4.000000, pos 3, 4 
        // Reservation: time 3.000000, pos 3, 3 
        // Reservation: time 2.000000, pos 3, 2 
        // Reservation: time 1.000000, pos 3, 1 

        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(2,2), 0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(!test_sim.planner->sensing_cone_occupied(SiteID(2,2), M_PI, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked

        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,1), M_PI / 2.0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(3,1), M_PI / 2.0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked

        IS_TRUE(!test_sim.planner->sensing_cone_occupied(SiteID(3,3), M_PI / 2.0, 2, sp.sensing_range, sp.sensing_angle)); // should not be blocked - facing away from agent 0
        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(3,3), M_PI / 2.0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(3,3), M_PI / 2.0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(2,1), M_PI / 2.0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked
        // IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(4,3), normalize(M_PI / 2.0), 2, sp.sensing_range, sp.sensing_angle)); // should be blocked


        // different direction
        test_sim.reset();
        test_sim.agents[0]->set_pos(SiteID(9, 5));
        test_sim.agents[0]->goal = SiteID(0, 5);
        test_sim.planner->reservations.clear();
        test_sim.planner->make_reservation(0, 9, 5, 0);
        test_sim.update();

        // Agent 0 looking for plan from start 9, 5 to goal 0, 5. Current time 0.000000...
        // Reservation: time 5.000000, pos 4, 5 
        // Reservation: time 4.000000, pos 5, 5 
        // Reservation: time 3.000000, pos 6, 5 
        // Reservation: time 2.000000, pos 7, 5 
        // Reservation: time 1.000000, pos 8, 5 
        // Reservation: time 0.000000, pos 0, 4 

        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(6,4), normalize(-M_PI / 2.0), 2, sp.sensing_range, sp.sensing_angle)); // this was the bug!
        IS_TRUE(in_vision_cone(Pose(4, 0.8, 0, 3.142), Pose(2.4, -0.8, 0, -1.571), sp.sensing_range, sp.sensing_angle).in_cone);

        // another different direction
        test_sim.reset();
        test_sim.agents[0]->set_pos(SiteID(9, 0));
        test_sim.agents[0]->goal = SiteID(0, 9);
        test_sim.planner->reservations.clear();
        test_sim.planner->make_reservation(0, 9, 0, 0);
        test_sim.update();

        // Reservation: time 5.000000, pos 4, 5 
        // Reservation: time 4.000000, pos 5, 4 
        // Reservation: time 3.000000, pos 6, 3 
        // Reservation: time 2.000000, pos 7, 2 
        // Reservation: time 1.000000, pos 8, 1 
        // Reservation: time 0.000000, pos 5, 3 

        printf("test starts here...\n");
        IS_TRUE(test_sim.agents[0]->goal == SiteID(0, 9));
        printf("agent goal is: %i, %i\n", test_sim.agents[0]->goal.idx, test_sim.agents[0]->goal.idy);
        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(7,3), normalize(M_PI / 4.0), 2, sp.sensing_range, sp.sensing_angle)); // another bug
        IS_TRUE(in_vision_cone(Pose(4, -4, 0, 2.356), Pose(4, -2.4, 0, M_PI / 4.0), sp.sensing_range, sp.sensing_angle).in_cone);
        IS_TRUE(test_sim.planner->reserved(2, 7, 2));

        // another case that should give me bad behavior (saw it while running a sim)
        test_sim.reset();
        test_sim.planner->reservations.clear();
        test_sim.planner->make_reservation(8, 7, 7, 0);
        test_sim.planner->make_reservation(9, 8, 8, 0);
        test_sim.planner->make_reservation(7, 6, 7, 0);
        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(6,6), normalize(M_PI / 2.0), 8.0, sp.sensing_range, sp.sensing_angle)); // another bug
        IS_TRUE(test_sim.planner->is_invalid_step(SiteID(6,6), SiteID(6,7), 8, sp.sensing_range, sp.sensing_angle))
        // test_sim.agents[0].set_pos(SiteID(8, 4));
        // test_sim.agents


        test_sim.reset();
        test_sim.planner->reservations.clear();
        test_sim.timestep = 79.0;
        test_sim.agents[0]->set_pos(SiteID(8, 6));
        test_sim.planner->make_reservation(79, 8, 6, 0);
        test_sim.agents[0]->goal = SiteID(7, 0);
        test_sim.update();
        // test_sim.planner->make_reservation(80, 7, 5, 1);
        // test_sim.planner->make_reservation(81, 7, 4, 1);
        // test_sim.planner->make_reservation(82, 7, 3, 1);
        // test_sim.planner->make_reservation(83, 7, 2, 1);
        // test_sim.planner->make_reservation(84, 7, 1, 1);
        // test_sim.planner->make_reservation(85, 7, 0, 1);
        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(6, 2), -1, 82, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(test_sim.planner->sensing_cone_invalid(SiteID(6, 2), 2.3561, 82, sp.sensing_range, sp.sensing_angle));







        // periodic simulation for testing periodic sensing
        sp.periodic = true;
        sp.cells_per_side = 10;
        AStarManager p_sim = AStarManager(sp);
        p_sim.planner->reservations[AStarPlanner::Reservation(5, 0, 0)] = -1;
        IS_TRUE(p_sim.planner->sensing_cone_occupied(SiteID(0, 9), M_PI / 2.0, 5, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(!p_sim.planner->sensing_cone_occupied(SiteID(0, 9), -M_PI / 2.0, 5, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(p_sim.planner->sensing_cone_occupied(SiteID(9, 0), 0, 5, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(!p_sim.planner->sensing_cone_occupied(SiteID(9, 0), M_PI, 5, sp.sensing_range, sp.sensing_angle));
        IS_TRUE(p_sim.planner->sensing_cone_occupied(SiteID(9, 9), M_PI / 4.0, 5, 2 * sp.sensing_range, sp.sensing_angle));
        // IS_TRUE(!p_sim.planner->sensing_cone_occupied(SiteID(9, 9), M_PI, 5, sp.sensing_range, sp.sensing_angle));






        // Fl_Window win(900, 900, "A* Pathfinding Test Case");
        // Canvas gui = Canvas(&test_sim, 0,0, win.w(), win.h());

        // win.resizable(&gui);

        // // win.end();
        // win.show();
        // gui.startAnimation();
        // return Fl::run();

    }



    {
        // set two agents on course to collide
        // sim.agents[0]->set_pos(SiteID(3, 0));
        // sim.agents[0]->goal = SiteID(3, sim.sp.cells_per_side - 1);
        // sim.agents[1]->set_pos(SiteID(3, sim.sp.cells_per_side - 1));
        // sim.agents[1]->goal = SiteID(3,0);


        // set agents on course to criss cross
        // sim.agents[0]->set_pos(SiteID(3, 0));
        // sim.agents[0]->goal = SiteID(6,4);
        // sim.agents[1]->set_pos(SiteID(6,0));
        // sim.agents[1]->goal = SiteID(2,4);


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