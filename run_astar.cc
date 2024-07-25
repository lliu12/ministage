#include "astar_utils.hh"
#include "astar_manager.hh"
#include "astar_canvas.hh"


int main(int argc, char* argv[])
{
    printf("running run_astar.cc\n");

    sim_params sp;

    sp.num_agents = 5;
    sp.periodic = false;
    sp.diags = true;
    sp.r_upper = 8;

    sp.cells_per_side = 20;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 2;

    sp.dt = .1;
    sp.time_steps = 200;

    sp.gui_speedup = 0.25; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = false;
    sp.gui_random_colors = true;

    // 
    Pose pose(0,0,0,0);

    AStarManager sim = AStarManager(sp);

    std::cout << "Checking diagonal distance heuristic function..." << std::endl;
    {
        IS_TRUE(nearest_periodic(Pose(0, 1, 0, 0), Pose(8, 3, 0, 0), 5.0) == Pose(-2,3,0,0));
        IS_TRUE(nearest_periodic(Pose(0, 1, 0, 0), Pose(1, 3, 0, 0), 5.0) == Pose(1,3,0,0));
        IS_TRUE(nearest_periodic(Pose(1, 8, 0, 0), Pose(9, 0, 0, 0), 5.0) == Pose(-1,10,0,0));
        IS_TRUE(nearest_periodic(Pose(1, 1, 0, 0), Pose(1, 4, 0, 0), 2.5) == Pose(1,-1,0,0));

        // nearest_periodic(Pose(0, 1, 0, 0), Pose(8, 3, 0, 0), sim.space->cells_per_side / 2.0).Print("nearest periodic test: ");

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
        AStarManager test_sim = AStarManager(sp);

        
        test_sim.agents[0]->set_pos(SiteID(3, 0));
        test_sim.agents[0]->goal = SiteID(3, 9);
        test_sim.update();

        // for (SiteID s : test_sim.agents[0]->plan) {
        //     printf("plan step %i, %i \n", s.idx, s.idy);
        // }

        // printf("is reserved? %i", static_cast<int>(test_sim.planner->reservations[9][0][9]));


// Reservation: time 9, pos 3, 9 
// Reservation: time 8, pos 3, 8 
// Reservation: time 7, pos 3, 7 
// Reservation: time 6, pos 3, 6 
// Reservation: time 5, pos 3, 5 
// Reservation: time 4, pos 3, 4 
// Reservation: time 3, pos 3, 3 
// Reservation: time 2, pos 3, 2 
// Reservation: time 1, pos 3, 1 

        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,2), 0, 2, sp.sensing_range, sp.sensing_angle)); // should be blocked
        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,2), M_PI / 2.0, 3, sp.sensing_range, sp.sensing_angle)); // should be blocked

        IS_TRUE(test_sim.planner->sensing_cone_occupied(SiteID(3,5), normalize(M_PI / 2.0 * 3), 4, sp.sensing_range, sp.sensing_angle)); // should be blocked

    }



    {
        // set two agents on course to collide
        sim.agents[0]->set_pos(SiteID(3, 0));
        sim.agents[0]->goal = SiteID(3, 9);
        sim.agents[1]->set_pos(SiteID(3, 9));
        sim.agents[1]->goal = SiteID(3,0);
    }

    {
    printf("Try opening a GUI window...\n");
    // sim.reset();

    Fl_Window win(700, 700, "A* Pathfinding");
    Canvas gui = Canvas(&sim, 0,0, win.w(), win.h());

    win.resizable(&gui);

    // win.end();
    win.show();
    gui.startAnimation();
    return Fl::run();
    }
}