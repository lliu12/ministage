// This is a more lightweight, customized version of RTV's Stage simulation.
#include <chrono>
#include "simulation_manager.hh"
#include "canvas.hh"


const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }

int main(int argc, char* argv[])
{
    // Tests that MiniStage is working as expected
    sim_params sp;

    sp.num_agents = 128;

    sp.periodic = false;
    sp.circle_arena = false;
    sp.r_upper = 8;
    sp.r_lower = 0;

    sp.sensing_angle = M_PI * 3.0 / 4.0;
    sp.sensing_range = 1;

    sp.cells_range = 10;
    sp.cells_per_side = floor(2.0 * sp.cells_range / sp.sensing_range); // 15;
    sp.cell_width = 2.0 * sp.cells_range / sp.cells_per_side;
    sp.use_sorted_agents = false;
    sp.use_cell_lists = true;
    
    sp.anglenoise = 1.5;
    sp.anglebias = 0;

    sp.avg_runsteps = 40;
    sp.randomize_runsteps = true;

    sp.cruisespeed = 0.6;
    sp.turnspeed = 3;
    sp.goal_tolerance = 0.3;

    sp.dt = .1;

    sp.gui_speedup = 5; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui

    sp.verbose = false;

    IS_TRUE(2 * sp.cells_range / sp.cells_per_side >= sp.sensing_range);
    // IS_TRUE(2 * sp.cells_range / sp.cells_per_side >= sp.dt * sp.cruisespeed;

    SimulationManager sim = SimulationManager(sp);

   int num_agents = sp.num_agents;

   // Run sanity checks
   // If parameter is not true, test fails

    std::cout << "Checking in_vision_cone function..." << std::endl;
    {
        // basic
        Pose p = Pose(0,0,0,0);
        Pose q = Pose(5.1, 0, 0, 0);
        cone_result cr = in_vision_cone(p, q, 5, M_PI);
        // std::cout << cr.dist_away << std::endl;
        IS_TRUE(cr.dist_away == p.Distance(q));
        IS_TRUE(!cr.in_cone);
        IS_TRUE(!in_vision_cone(p,q,5.1, M_PI).in_cone);
        IS_TRUE(in_vision_cone(p,q,5.2, M_PI).in_cone);
        IS_TRUE(!in_vision_cone(p,q,5.2, 0).in_cone);

        // try some translation
        p = Pose(6,6,0,0);
        q = Pose(11.1, 6, 0, 0);
        cr = in_vision_cone(p, q, 5, M_PI);
        IS_TRUE(cr.dist_away == p.Distance(q));
        IS_TRUE(!cr.in_cone);
        IS_TRUE(!in_vision_cone(p,q,5.1, M_PI).in_cone);
        IS_TRUE(in_vision_cone(p,q,5.2, M_PI).in_cone);
        IS_TRUE(!in_vision_cone(p,q,5.2, 0).in_cone);

        // try some rotation
        p = Pose(-2,-2,0,0);
        q = Pose(-1, -1, 0, 0);
        cr = in_vision_cone(p, q, 1.5, M_PI);
        IS_TRUE(cr.dist_away == p.Distance(q));
        IS_TRUE(cr.in_cone);
        IS_TRUE(!in_vision_cone(p,q,1, M_PI).in_cone);
        IS_TRUE(!in_vision_cone(p,q,5, M_PI/2).in_cone);
        IS_TRUE(in_vision_cone(p,q,5, 1.1 * M_PI / 2).in_cone);

    }

    printf("Testing position sort neighbor-finding...\n");
    if (sp.use_sorted_agents)
    {
        sim.reset();
        for (int k = 0; k<5; k++) {

            printf("Testing sorting (step k = %i)...\n", k);
            // after we run sd->update, positions should be sorted
            // bool sorted = sim.sd->vecs_sorted();
            Agent *begin_agent = *sim.sd->agents_byx_vec.begin();
            double last_x = begin_agent->get_pos().x;
            for (Agent *a : sim.sd->agents_byx_vec) {
                IS_TRUE(last_x <= a->get_pos().x);
                last_x = a->get_pos().x;
                // a->get_pos().Print("Sorted agents by x: ");
            }

            // sanity check that the lower_bound returned exists and is >= the desired edge
            // printf("Testing narrowing down nearby neighbors (step k = %i)...\n", k);
            Agent edge(-1, &sp, sim.sd); // dummy model used to find bounds in the sets

            for (Agent *a : sim.agents) {
                Pose gp = a->get_pos();
                std::vector<Agent *>::iterator xmin;
                edge.set_pos(Pose(gp.x - sim.sp.sensing_range, gp.y, 0, 0)); // LEFT
                xmin = std::lower_bound(sim.sd->agents_byx_vec.begin(), sim.sd->agents_byx_vec.end(), &edge, SimulationData::ltx());
                
                Agent *xminAgent = *xmin;
                IS_TRUE(xminAgent->get_pos().x >= edge.get_pos().x);
                IS_TRUE(xmin != sim.sd->agents_byx_vec.end());
                // xminAgent->cur_pos->Print("");
            }

            sim.update();
            sim.sd->update();
        }
    }



    printf("Testing cell list neighbor-finding...\n");
    if (sp.use_cell_lists)
    {
        sim.reset();

        printf("Testing cell construction...\n");
        if (!sp.periodic) {
            for (int idx = 0; idx < sp.cells_per_side; idx++) {
                for (int idy = 0; idy < sp.cells_per_side; idy++) {
                    if (!sim.sd->cells[idx][idy]->is_outer_cell) {
                        IS_TRUE(sim.sd->cells[idx][idy]->neighbors.size() == 8); // inner cells should have 8 neighbors
                    }
                    else if ((idx != 0 && idx < sp.cells_per_side - 1) || (idy != 0 && idy < sp.cells_per_side - 1)) {
                        IS_TRUE(sim.sd->cells[idx][idy]->neighbors.size() == 6); // outer non-corner cells should have 6 neighbors (5 + overflow cell)
                    }
                    else {
                        IS_TRUE(sim.sd->cells[idx][idy]->neighbors.size() == 4); // corner cells should have 4 neighbors (3 + overflow cell)
                    }
                }
            }
        }


        // printf("Testing get cell for pos function...\n");
        // printf("cell width: %f", sp.cell_width);
        // Pose p = Pose(-18, -9.9, 0, 0);
        // sim.sd->get_cell_for_pos(&p);

    }

    if(1) {
        // try running a trial
        printf("Try running a trial...\n");
        auto start_time = std::chrono::high_resolution_clock::now();
        sim.reset();
        sim.run_trials(10, 1200);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        std::cout << "Time taken to run: " << duration.count() << " seconds" << std::endl;
    }


    if (1) {   
        // try opening a GUI window
        printf("Try opening a GUI window...\n");
        sim.reset();

        Fl_Window win(500, 500, "MiniStage");
        Canvas gui = Canvas(&sim, 0,0, win.w(), win.h());

        win.resizable(&gui);

        // win.end();
        win.show();
        gui.startAnimation();
        return Fl::run();
    }
    else {return 0;}

}