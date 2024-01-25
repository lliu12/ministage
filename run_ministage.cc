// This is a more lightweight, customized version of RTV's Stage simulation.
#include <chrono>
#include "ministage.hh"
#include "canvas.hh"


int main(int argc, char* argv[])
{
    // Tests that MiniStage is working as expected
    sim_params sp;

    sp.num_agents = 128;
    sp.periodic = false;
    sp.circle_arena = false;
    sp.r_upper = 8;
    sp.r_lower = 0;
    sp.verbose = false;
    sp.anglenoise = 0;
    sp.anglebias = 0;
    sp.sensing_angle = M_PI / 2.0; // maybe switch everything to work just in degrees????
    sp.sensing_range = 1;
    sp.cruisespeed = 0.6;
    sp.turnspeed = 0.8;
    sp.goal_tolerance = 0.4;
    sp.dt = .1;
    sp.gui_speedup = 1; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui

    SimulationManager sim = SimulationManager(sp);

   int num_agents = sp.num_agents;

   // Run sanity checks
   // If parameter is not true, test fails
    #define IS_TRUE(x) { if (!(x)) std::cout << __FUNCTION__ << " failed on line " << __LINE__ << std::endl; }

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

    printf("Testing nearest neighbor search...\n");
    {
        for (int k = 0; k<5; k++) {

            sim.sd->update(sim.agents);

            printf("Testing fiducial sorting...\n");
            // bool sorted = sim.sd->vecs_sorted();
            Agent *begin_agent = *sim.sd->agents_byx_vec.begin();
            double last_x = begin_agent->get_pos().x;
            for (Agent *a : sim.sd->agents_byx_vec) {
                IS_TRUE(last_x <= a->get_pos().x);
                last_x = a->get_pos().x;
                // a->get_pos().Print("Sorted agents by x: ");
            }

            printf("Testing narrowing down nearby neighbors...\n");
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
        }
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

        // win.resizable(&gui);

        // win.end();
        win.show();

        gui.startAnimation();

        return Fl::run();
    }
    else {return 0;}

}