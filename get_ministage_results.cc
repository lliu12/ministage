// Running this script produces the local sensing simulation data used in 
// Main Text Fig. 2 of "Noise-Enabled Goal Attainment in Crowded Collectives"

#include <chrono>
#include "simulation_manager.hh"
#include "canvas.hh"

const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }


int main(int argc, char* argv[])
{

    sim_params sp;
    double sim_run_length = 8000;

    std::vector<bool> periodic_arr{true}; 

    std::vector<int> num_agents_arr = { 16,  20,  24,  28,  32,  36,  40,  44,  48,  52,  56,  60,  64,
                                        68,  72,  76,  80,  84,  88,  92,  96, 100, 104, 108, 112, 116,
                                        120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164, 168,
                                        172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216, 220,
                                        224, 228, 232, 236, 240, 244, 248, 252, 256};


    std::vector<float> noise_arr = {0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1. , 1.1, 1.2,
       1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2., 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0};

    sp.anglebias = 0;

    // name outfile and save headings
    sp.outfile_name = "20250321_long_out.txt";
    std::ofstream agents_file_head(sp.outfile_name, std::ios::out);
    agents_file_head << "trial, periodic, num_robots, noise, noise_prob, sim_time, robot_id, x_pos, y_pos, angle, goal_x_pos, goal_y_pos, goal_birth_time, goals_reached, stopped, nearby_robot, addtl_data\n";
    agents_file_head.close();


    sp.save_data_interval = 1000.0;
    sp.turnspeed = -1; // -1 for instant turning
    sp.circle_arena = false;

    int num_trials_high_variance = 50;
    int num_trials_low_variance = 20; // run fewer trials for regions where the variance low (as we saw from previous runs)

    // parameters to leave unchanged
    sp.r_upper = 20;
    sp.r_lower = 0;

    sp.noise_prob = 1.0;
    sp.conditional_noise = false;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 2;

    sp.cells_range = 50; // only used if not periodic
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
    sp.gui_random_colors = false;
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


                if(sp.periodic) { sp.cells_range = sp.r_upper; }
                sp.cells_per_side = floor(2.0 * sp.cells_range / sp.sensing_range);
                sp.cell_width = 2.0 * sp.cells_range / sp.cells_per_side;

                auto this_start_time = std::chrono::high_resolution_clock::now();

                SimulationManager sim = SimulationManager(sp);


                int num_trials = (sp.num_agents <= 128 && sp.anglenoise <= 2.0) ? num_trials_high_variance : num_trials_low_variance;
                sim.run_trials(num_trials, sim_run_length);


                auto this_end_time = std::chrono::high_resolution_clock::now();
                auto this_duration = std::chrono::duration_cast<std::chrono::milliseconds>(this_end_time - this_start_time);

                complete += 1;
                printf("Just ran World %i / %i in %lli milliseconds: periodic %i, robots %i, noise %f \n", complete, total_worlds, this_duration.count(), p, num, noise);
                

            }
        }
    }
    auto all_end_time = std::chrono::high_resolution_clock::now();
    auto all_duration = std::chrono::duration_cast<std::chrono::seconds>(all_end_time - all_start_time);
    std::cout << "\nTime taken to run all trials: " << all_duration.count() << " seconds" << std::endl;

}