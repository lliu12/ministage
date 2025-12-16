// Running this script produces the simulation data used in
// Main Text Fig. 3 of "Noise-Enabled Goal Attainment in Crowded Collectives"

#include <chrono>
#include "simulation_manager.hh"
#include "canvas.hh"
#include <map>
#include <utility> 
#include <filesystem>


const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }


int main(int argc, char* argv[])
{

    sim_params sp;
    double sim_run_length = 300;

    std::vector<bool> periodic_arr{false};
    std::filesystem::path base_dir = SIM_DATA_DIR;
    std::filesystem::create_directories(base_dir);
    sp.outfile_name = (base_dir / "fig3_simulation_data.txt").string();

    std::ofstream agents_file_head(sp.outfile_name, std::ios::out);
    agents_file_head << "trial,periodic,num_robots,noise,noise_prob,sim_time,robot_id,x_pos,y_pos,angle,goal_x_pos,goal_y_pos,goal_birth_time,goals_reached,stopped,nearby_robot,addtl_data\n";
    agents_file_head.close();

    sp.save_data_interval = 75;
    sp.circle_arena = false;
    sp.dt = .1;
    sp.anglebias = 0;
    int num_trials = 100;

    // Each row represents a noise, num robots, avg fwd speed, avg turn speed combo from analyzing experimental data 
    std::map<std::pair<double, int>, std::pair<double, double>> speeds_lookup = {
        { { 0.0, 1 }, { 0.1415, 1.5784 } },
        { { 0.0, 3 }, { 0.1253, 1.7059 } },
        { { 0.0, 5 }, { 0.1062, 0.8778 } },
        { { 0.0, 7 }, { 0.1244, 1.4746 } },
        { { 0.0, 10 }, { 0.0633, 1.2407 } },
        { { 0.0, 15 }, { 0.0894, 0.7821 } },
        { { 0.0, 20 }, { 0.1105, 0.6799 } },
        { { 0.25, 3 }, { 0.1113, 0.7576 } },
        { { 0.25, 5 }, { 0.0986, 0.538 } },
        { { 0.5, 1 }, { 0.1144, 1.3703 } },
        { { 0.5, 3 }, { 0.1033, 1.0436 } },
        { { 0.5, 5 }, { 0.1024, 0.86 } },
        { { 0.5, 7 }, { 0.0981, 0.6719 } },
        { { 0.5, 10 }, { 0.0934, 0.5658 } },
        { { 0.5, 15 }, { 0.0802, 0.47 } },
        { { 0.5, 20 }, { 0.0772, 0.4543 } },
        { { 0.75, 3 }, { 0.1031, 1.234 } },
        { { 0.75, 5 }, { 0.1016, 1.1159 } },
        { { 0.75, 7 }, { 0.0998, 0.9773 } },
        { { 0.75, 10 }, { 0.0923, 0.8195 } },
        { { 0.75, 15 }, { 0.0853, 0.6792 } },
        { { 1.0, 1 }, { 0.1132, 1.377 } },
        { { 1.0, 5 }, { 0.0997, 1.1936 } },
        { { 1.0, 7 }, { 0.1004, 1.1341 } },
        { { 1.0, 10 }, { 0.092, 0.9924 } },
        { { 1.0, 15 }, { 0.0877, 0.8389 } },
        { { 1.0, 20 }, { 0.0886, 0.902 } },
        { { 1.25, 10 }, { 0.0941, 1.0651 } },
        { { 1.5, 1 }, { 0.1109, 1.5599 } },
        { { 1.5, 3 }, { 0.1021, 1.4416 } },
        { { 1.5, 5 }, { 0.0997, 1.3235 } },
        { { 1.5, 7 }, { 0.0983, 1.2635 } },
        { { 1.5, 10 }, { 0.0934, 1.1584 } },
        { { 1.5, 15 }, { 0.0921, 1.098 } },
        { { 1.5, 20 }, { 0.0908, 1.1239 } },
    };

    // parameters to leave unchanged
    sp.r_upper = .6;
    sp.r_lower = 0;

    sp.noise_prob = 1.0;
    sp.conditional_noise = false;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    // sp.sensing_range = 0.2;
    sp.sensing_range = 0.1564;

    sp.cells_range = 1; // only used if not periodic
    if(sp.periodic) { sp.cells_range = sp.r_upper; }
    sp.cells_per_side = floor(2.0 * sp.cells_range / sp.sensing_range);
    sp.cell_width = 2.0 * sp.cells_range / sp.cells_per_side;
    sp.use_sorted_agents = false;
    sp.use_cell_lists = true;
    

    sp.avg_runsteps = 15;
    sp.randomize_runsteps = true;
    sp.goal_tolerance = 0.08; // sp.avg_runsteps * sp.dt * sp.cruisespeed;
    sp.gui_speedup = 25; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = false;
    sp.verbose = false;

    IS_TRUE(2 * sp.cells_range / sp.cells_per_side >= sp.sensing_range);

    int total_worlds = speeds_lookup.size();
    int complete = 0;
    auto all_start_time = std::chrono::high_resolution_clock::now();
    for (bool p : periodic_arr) {
        sp.periodic = p;

        for (const auto& entry : speeds_lookup) {
            sp.anglenoise      = entry.first.first;
            sp.num_agents    = entry.first.second;
            sp.cruisespeed  = entry.second.first;
            sp.turnspeed = entry.second.second;

            auto this_start_time = std::chrono::high_resolution_clock::now();

            SimulationManager sim = SimulationManager(sp);
            sim.run_trials(num_trials, sim_run_length);


            auto this_end_time = std::chrono::high_resolution_clock::now();
            auto this_duration = std::chrono::duration_cast<std::chrono::seconds>(this_end_time - this_start_time);

            complete += 1;
            printf("Just ran World %i / %i in %lli seconds: periodic %i, robots %i, noise %f \n", complete, total_worlds, this_duration.count(), sp.periodic, sp.num_agents, sp.anglenoise);    
        }
    }
    auto all_end_time = std::chrono::high_resolution_clock::now();
    auto all_duration = std::chrono::duration_cast<std::chrono::seconds>(all_end_time - all_start_time);
    std::cout << "\nTime taken to run all trials: " << all_duration.count() << " seconds" << std::endl;

}