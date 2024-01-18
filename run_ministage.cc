// This is a more lightweight, customized version of RTV's Stage simulation.
#include "ministage.hh"

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

    sp.num_agents = 5;
    sp.periodic = false;
    sp.circle_arena = false;
    sp.r_upper = 8;
    sp.r_lower = 0;
    sp.verbose = false;
    sp.dt = 1;

    SimulationManager sim = SimulationManager(sp);

   int num_agents = sp.num_agents;

   // Run sanity checks

   // If parameter is not true, test fails
    #define IS_TRUE(x) { if (!(x)) std::cout << __FUNCTION__ << " failed on line " << __LINE__ << std::endl; }

   std::cout << "Checking initial pose generation..." << std::endl;
    for (int i = 0; i < num_agents; i++) 
    {
        // sim.agents[i]->cur_pos->Print("Random Pose: ");
        // std::cout << sim.agents[i]->cur_pos << std::endl;
        IS_TRUE(sim.agents[i]->cur_pos == sim.sd.positions[i]); // simdata and agent pose pointers match
        IS_TRUE(*sim.agents[i]->cur_pos == *sim.sd.positions[i]); // simdata and agent pose values match
    }

    std::cout << "Checking case where agent updates Poses..." << std::endl;
    for (int i = 0; i < num_agents; i++) 
    {
        sim.agents[i]->gen_start_goal_positions();
        IS_TRUE(sim.agents[i]->cur_pos == sim.sd.positions[i]); // simdata and agent pose pointers match
        IS_TRUE(*sim.agents[i]->cur_pos == *sim.sd.positions[i]); // simdata and agent pose values match
    }

    std::cout << "Checking case where simdata updates Poses..." << std::endl;
    for (int i = 0; i < num_agents; i++) 
    {
        *sim.sd.positions[i] = Pose::Random(0, 1, 0, 1);
        IS_TRUE(sim.agents[i]->cur_pos == sim.sd.positions[i]); // simdata and agent pose pointers match
        IS_TRUE(*sim.agents[i]->cur_pos == *sim.sd.positions[i]); // simdata and agent pose values match
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

    // try running a trial
    sim.run_trial(20);


    return 0;
}

// Run multiple trials of: 
// Create world (Inside world: initialize positions, set up memoized locations by grid, etc.)

// Simulate robot behavior (Two update steps, one to update sensors, one to update positions)

// GUI?? 

// Save data (Inside world? )

// Clean up to avoid memory leaks!