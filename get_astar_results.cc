// Sep 5, 2024
// First run of AStar data collection

#include <chrono>
#include "astar_utils.hh"
#include "astar_manager.hh"
#include "astar_canvas.hh"

const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }


int main(int argc, char* argv[])
{

    sim_params sp;
    double sim_run_length = 1000;

    std::vector<bool> periodic_arr{false}; // periodic_arr{true, false};
    std::vector<int> num_agents_arr = {20, 40, 60, 80, 100};

    sp.outfile_name = "astar_20240905_out.txt";
    sp.save_data_interval = 10.0;
    sp.addtl_data = "astar";
    int num_trials = 5;


    sp.diags = true;
    sp.r_upper = 8;
    sp.diags_take_longer = true;

    sp.cells_per_side = 20;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 1.5; // 2 * 1.25; // larger value for cps = 10

    sp.dt = .1;
    sp.time_steps = 800;
    
    sp.gui_speedup = sp.diags_take_longer ? 1.25 : 0.75 ; // speed up gui compared to real time
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = false;
    sp.gui_random_colors = true;



    int total_worlds = periodic_arr.size() * num_agents_arr.size();
    int complete = 0;
    auto all_start_time = std::chrono::high_resolution_clock::now();
    for (bool p : periodic_arr) {
        for (int num : num_agents_arr) {
                sp.periodic = p;
                sp.num_agents = num;

                auto this_start_time = std::chrono::high_resolution_clock::now();

            AStarManager sim = AStarManager(sp);
                sim.run_trials(num_trials, sim_run_length);

                auto this_end_time = std::chrono::high_resolution_clock::now();
                auto this_duration = std::chrono::duration_cast<std::chrono::seconds>(this_end_time - this_start_time);

                complete += 1;
            // printf("Just ran World %i / %i in %lli seconds: periodic %i, robots %i \n", complete, total_worlds, this_duration.count(), p, num);
            printf("\x1B[36mJust ran World %i / %i in %lli seconds: periodic %i, robots %i \n\033[0m\t\t", complete, total_worlds, this_duration.count(), p, num);
        }
    }
    auto all_end_time = std::chrono::high_resolution_clock::now();
    auto all_duration = std::chrono::duration_cast<std::chrono::seconds>(all_end_time - all_start_time);
    std::cout << "\nTime taken to run all trials: " << all_duration.count() << " seconds" << std::endl;

}