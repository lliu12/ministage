// Aug 23, 2024
// To help test out the analytical approximations, I make the step size smaller, increase the range of noises, and test fewer values for swarm size n
// Changed both run steps and cruise speed. Step size is run steps * dt * cruise speed (?)
// Also changed goal tolerance to be one step length

// Aug 26, 2024
// Made data save 10x less frequent, arena huge (from L=16 to L=40), run simulation for several times longer, make vision cone much larger to account for larger arena, 
// And make steps half as long (now only 10 runsteps on average)
// Don't run nonperiodic simulations
// And increase goal tolerance

#include <chrono>
#include "simulation_manager.hh"
#include "canvas.hh"

const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }


int main(int argc, char* argv[])
{

    sim_params sp;
    double sim_run_length = 3000;

    std::vector<bool> periodic_arr{true}; // periodic_arr{true, false};
    std::vector<int> num_agents_arr = {16, 64, 128, 256};
    std::vector<float> noise_arr = {-1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1. , 1.1, 1.2,
       1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2., 2.5, 3.0};

    
    // sp.periodic = false;
    // sp.num_agents = 200;
    // sp.anglenoise = -1; // -1 for uniform random noise, otherwise this is the STD of gaussian noise 
    // sp.anglebias = 0;
    sp.outfile_name = "20240826_out.txt";
    sp.save_data_interval = 1000.0;
    sp.turnspeed = -1; // -1 for instant turning, 10 is a good value otherwise for visuals
    sp.circle_arena = true;
    int num_trials = 5;


    // parameters to leave unchanged
    sp.r_upper = 20;
    sp.r_lower = 0;

    sp.noise_prob = 1.0;
    sp.conditional_noise = false;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 2;

    sp.cells_range = 50; // only used if not periodic
    if(sp.periodic) { sp.cells_range = sp.r_upper; }
    sp.cells_per_side = floor(2.0 * sp.cells_range / sp.sensing_range);
    sp.cell_width = 2.0 * sp.cells_range / sp.cells_per_side;
    sp.use_sorted_agents = false;
    sp.use_cell_lists = true;
    

    sp.avg_runsteps = 10;
    sp.randomize_runsteps = true;

    sp.cruisespeed = 0.5;
    sp.dt = .1;

    sp.goal_tolerance = 0.6; // sp.avg_runsteps * sp.dt * sp.cruisespeed;

    

    sp.gui_speedup = 25; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = false;
    sp.verbose = false;

    IS_TRUE(2 * sp.cells_range / sp.cells_per_side >= sp.sensing_range);

    int total_worlds = periodic_arr.size() * num_agents_arr.size() * noise_arr.size();
    int complete = 0;
    auto all_start_time = std::chrono::high_resolution_clock::now();
    for (bool p : periodic_arr) {
        for (int num : num_agents_arr) {
            for (float noise : noise_arr) {
                sp.periodic = p;
                sp.num_agents = num;
                sp.anglenoise = noise;

                auto this_start_time = std::chrono::high_resolution_clock::now();

                SimulationManager sim = SimulationManager(sp);
                sim.run_trials(num_trials, sim_run_length);


                auto this_end_time = std::chrono::high_resolution_clock::now();
                auto this_duration = std::chrono::duration_cast<std::chrono::seconds>(this_end_time - this_start_time);

                complete += 1;
                printf("Just ran World %i / %i in %lli seconds: periodic %i, robots %i, noise %f \n", complete, total_worlds, this_duration.count(), p, num, noise);
                

            }
        }
    }
    auto all_end_time = std::chrono::high_resolution_clock::now();
    auto all_duration = std::chrono::duration_cast<std::chrono::seconds>(all_end_time - all_start_time);
    std::cout << "\nTime taken to run all trials: " << all_duration.count() << " seconds" << std::endl;

}