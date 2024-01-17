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

    return 0;
}

// Run multiple trials of: 
// Create world (Inside world: initialize positions, set up memoized locations by grid, etc.)

// Simulate robot behavior (Two update steps, one to update sensors, one to update positions)

// GUI?? 

// Save data (Inside world? )

// Clean up to avoid memory leaks!