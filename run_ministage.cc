// This is a more lightweight, customized version of RTV's Stage simulation.
#include "ministage.hh"
#include "canvas.hh"

int main(int argc, char* argv[])
{
    
//   if (argc < 2) {
//     // report version
//     std::cout << argv[0] << " Version " << Tutorial_VERSION_MAJOR << "."
//               << Tutorial_VERSION_MINOR << std::endl;
//     std::cout << "Usage: " << argv[0] << " number" << std::endl;
//     return 1;
//   }
//   // convert input to double
//   const double inputValue = std::stod(argv[1]);



    // Tests that MiniStage is working as expected

    std::cout << "Hello, MiniStage!" << std::endl;

    sim_params sp;

    sp.num_agents = 10;
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

   std::cout << "Checking initial pose generation..." << std::endl;
    for (int i = 0; i < num_agents; i++) 
    {
        // sim.agents[i]->get_pos().Print("");
        // sim.sd->positions[i]->Print("");
        // std::cout << sim.agents[i]->cur_pos << std::endl;
        // std::cout << sim.sd->positions[i] << std::endl;
        
        IS_TRUE(sim.agents[i]->cur_pos == sim.sd->positions[i]); // simdata and agent pose pointers match
        IS_TRUE(*sim.agents[i]->cur_pos == *sim.sd->positions[i]); // simdata and agent pose values match
    }

    std::cout << "Checking case where agent updates Poses..." << std::endl;
    for (int i = 0; i < num_agents; i++) 
    {
        sim.agents[i]->gen_start_goal_positions();
        IS_TRUE(sim.agents[i]->cur_pos == sim.sd->positions[i]); // simdata and agent pose pointers match
        IS_TRUE(*sim.agents[i]->cur_pos == *sim.sd->positions[i]); // simdata and agent pose values match
    }

    std::cout << "Checking case where simdata updates Poses..." << std::endl;
    for (int i = 0; i < num_agents; i++) 
    {
        *sim.sd->positions[i] = Pose::Random(0, 1, 0, 1);
        IS_TRUE(sim.agents[i]->cur_pos == sim.sd->positions[i]); // simdata and agent pose pointers match
        IS_TRUE(*sim.agents[i]->cur_pos == *sim.sd->positions[i]); // simdata and agent pose values match
    }

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

    std::cout << "Testing nearest neighbor search..." << std::endl;
    {
        for (int k = 0; k <2; k++) {

            sim.sd->update(sim.agents);
            // sim.update();

            std::cout << "Testing fiducial sorting..." << std::endl;
            Agent *begin_agent = *sim.sd->agents_byx.begin();
            double last_x = begin_agent->get_pos().x;
            for (Agent *a : sim.sd->agents_byx) {
                IS_TRUE(last_x <= a->get_pos().x);
                last_x = a->get_pos().x;
            }


            std::cout << "Testing narrowing down nearby neighbors..." << std::endl;
            Pose dummy_pose;
            Agent edge(-1, sp, sim.sd, &dummy_pose); // dummy model used to find bounds in the sets

            for (Agent *a : sim.agents) {
                Pose gp = a->get_pos();
                edge.set_pos(Pose(gp.x - sp.sensing_range, gp.y, 0, 0)); // LEFT
                std::set<Agent *, SimulationData::ltx>::iterator xmin = sim.sd->agents_byx.lower_bound(&edge);
                // Agent *xminAgent = *xmin;
                IS_TRUE(xmin != sim.sd->agents_byx.end());
            }

        }
    }

    if (1) {    
        // try running a trial
        std::cout << "Try running a trial..." << std::endl;
        sim.reset();
        sim.run_trial(20);



        // try opening a GUI window
        std::cout << "Try opening a GUI window..." << std::endl;
        sim.reset();


        Fl_Window win(500, 500, "Simple Canvas Example");
        Canvas gui = Canvas(sim, 0,0, win.w(), win.h());

        // win.resizable(&gui);

        // win.end();
        win.show();

        gui.startAnimation();

        return Fl::run();
    }
    else {return 0;}

}