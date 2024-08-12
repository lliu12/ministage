// May 22, 2024. I add conditional noise and only adding noise a percentage of the time.
// Script is modified from previous runs because the structure of the parameters we want to sweep through is different now.

#include <chrono>
#include "simulation_manager.hh"
#include "canvas.hh"


const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#ifndef IS_TRUE
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }
#endif


int main(int argc, char* argv[])
{

    sim_params sp;

    std::vector<bool> periodic_arr{true, false};
    std::vector<int> num_agents_arr = {16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256};


    std::vector<float> gaussian_noise_arr = {-1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1. , 1.1, 1.2,
       1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2. }; // run these all with conditional noise off, anglenoise varies, noise prob = 1

    std::vector<float> noise_prob_arr = {0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.}; // run these all with conditional noise off, anglenoise = -1 (unif noise), noise prob varies
    
    // finally do a run with anglenoise = -1, conditional noise on, noise prob = 1

    
    sp.anglebias = 0;
    sp.outfile_name = "20240522_out.txt";
    sp.save_data_interval = 100.0;
    sp.turnspeed = -1; // -1 for instant turning, 10 is a good value otherwise for visuals
    sp.circle_arena = true;
    int num_trials = 5;


    // parameters to leave unchanged
    sp.r_upper = 8;
    sp.r_lower = 0;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 0.6;

    sp.cells_range = 50; // only used if not periodic
    if(sp.periodic) { sp.cells_range = sp.r_upper; }
    sp.cells_per_side = floor(2.0 * sp.cells_range / sp.sensing_range);
    sp.cell_width = 2.0 * sp.cells_range / sp.cells_per_side;
    sp.use_sorted_agents = false;
    sp.use_cell_lists = true;
    

    sp.avg_runsteps = 40;
    sp.randomize_runsteps = true;

    sp.cruisespeed = 0.6;
    sp.goal_tolerance = 0.3;

    sp.dt = .1;

    sp.gui_speedup = 25; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = false;
    sp.verbose = false;

    IS_TRUE(2 * sp.cells_range / sp.cells_per_side >= sp.sensing_range);

    int total_worlds = periodic_arr.size() * num_agents_arr.size() * (gaussian_noise_arr.size() + noise_prob_arr.size() + 1);
    int complete = 0;
    auto all_start_time = std::chrono::high_resolution_clock::now();

    printf("Running constant Gaussian noise worlds...");
    for (bool p : periodic_arr) {
        for (int num : num_agents_arr) {
            for (float noise : gaussian_noise_arr) {
                sp.periodic = p;
                sp.num_agents = num;
                sp.anglenoise = noise;
                sp.noise_prob = 1.0;
                sp.conditional_noise = false;
                sp.addtl_data = "constant noise";

                auto this_start_time = std::chrono::high_resolution_clock::now();

                SimulationManager sim = SimulationManager(sp);
                sim.run_trials(num_trials, 1200);


                auto this_end_time = std::chrono::high_resolution_clock::now();
                auto this_duration = std::chrono::duration_cast<std::chrono::seconds>(this_end_time - this_start_time);

                complete += 1;
                printf("Just ran World %i / %i in %lli seconds: periodic %i, robots %i, noise %f \n", complete, total_worlds, this_duration.count(), p, num, noise);
                

            }
        }
    }


    printf("Running noise probability worlds...");
    for (bool p : periodic_arr) {
        for (int num : num_agents_arr) {
            for (float noise_prob : noise_prob_arr) {
                sp.periodic = p;
                sp.num_agents = num;
                sp.anglenoise = -1;
                sp.noise_prob = noise_prob;
                sp.conditional_noise = false;
                sp.addtl_data = "noise probability";

                auto this_start_time = std::chrono::high_resolution_clock::now();

                SimulationManager sim = SimulationManager(sp);
                sim.run_trials(num_trials, 1200);


                auto this_end_time = std::chrono::high_resolution_clock::now();
                auto this_duration = std::chrono::duration_cast<std::chrono::seconds>(this_end_time - this_start_time);

                complete += 1;
                printf("Just ran World %i / %i in %lli seconds: periodic %i, robots %i, noise prob %f \n", complete, total_worlds, this_duration.count(), p, num, noise_prob);
                

            }
        }
    }


    printf("Running conditional noise worlds...");
    for (bool p : periodic_arr) {
        for (int num : num_agents_arr) {
            sp.periodic = p;
            sp.num_agents = num;
            sp.anglenoise = -1;
            sp.noise_prob = 1;
            sp.conditional_noise = true;
            sp.addtl_data = "conditional noise";

            auto this_start_time = std::chrono::high_resolution_clock::now();

            SimulationManager sim = SimulationManager(sp);
            sim.run_trials(num_trials, 1200);


            auto this_end_time = std::chrono::high_resolution_clock::now();
            auto this_duration = std::chrono::duration_cast<std::chrono::seconds>(this_end_time - this_start_time);

            complete += 1;
            printf("Just ran World %i / %i in %lli seconds: periodic %i, robots %i, conditional noise \n", complete, total_worlds, this_duration.count(), p, num);
        }
    }


    auto all_end_time = std::chrono::high_resolution_clock::now();
    auto all_duration = std::chrono::duration_cast<std::chrono::seconds>(all_end_time - all_start_time);
    std::cout << "\nTime taken to run all trials: " << all_duration.count() << " seconds" << std::endl;

}