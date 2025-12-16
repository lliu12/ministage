// Running this script produces the global planner data used in 
// Main Text Fig. 4 of "Noise-Enabled Goal Attainment in Crowded Collectives"

#include <chrono>
#include <filesystem>
#include "astar_utils.hh"
#include "astar_manager.hh"
#include "astar_canvas.hh"

int main(int argc, char* argv[])
{
    sim_params sp;
    double sim_run_length = 8000;

    std::vector<bool> periodic_arr{true};
    std::vector<int> num_agents_arr = {1, 16, 32, 64, 96, 128}; // reset to this version before upload

    // initialize output files
    std::filesystem::path base_dir = SIM_DATA_DIR;
    std::filesystem::create_directories(base_dir);
    std::string projectname = (base_dir / "fig4_astar").string();
    std::string planner_filename = projectname + "_planner_data.txt"; // save per-trial info about planner function calls
    sp.outfile_name = projectname + "_agents_data.txt";


    // output file headings
    std::ofstream planner_file_head(planner_filename, std::ios::out);
    planner_file_head << "num_robots,periodic,trial,sim_step_time,search_call_count,is_invalid_step_call_count,replan_count,wallclock_ms_since_start\n";
    planner_file_head.close();

    std::ofstream agents_file_head(sp.outfile_name, std::ios::out);
    agents_file_head << "trial,periodic,num_robots,sim_step_time,robot_id,x_pos,y_pos,goal_birth_time,goals_reached,noise_type\n";
    agents_file_head.close();



    std::ofstream planner_file;
    planner_file << std::fixed << std::setprecision(2);
    planner_file.open(planner_filename, std::ios_base::app);

    int wall_clock_save_data_interval = 1000.; // wall clock time
    sp.addtl_data = "astar";
    int num_trials = 20;


    sp.diags = true;
    sp.r_upper = 20;
    sp.diags_take_longer = true;

    sp.cells_per_side = 30;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 2.;


    float cruisespeed = 0.5;
    sp.time_steps = sim_run_length * sp.cells_per_side * cruisespeed / (2 * sp.r_upper); // number of discrete a* timesteps to run for so that speed matches continuous simulation
    sp.save_data_interval = wall_clock_save_data_interval * sp.cells_per_side * cruisespeed / (2 * sp.r_upper); // in discrete planner steps

    sp.verbose = false;
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

                // in-planner outfile for saving data about agent positions
                sim.outfile << std::fixed << std::setprecision(2);
                sim.outfile.open(sp.outfile_name, std::ios_base::app);

                for (int i = 0; i < num_trials; i++) {
                    
                    auto trial_start_time = std::chrono::high_resolution_clock::now();

                    // sim.run_trial(sp.time_steps, i); // replace run_trial or run_trials with a code block that helps save extra data on runtime, etc
                    {
                        sim.reset();

                        while (sim.timestep < sp.time_steps) {
                            if (!sp.outfile_name.empty() && fmod(sim.timestep, sp.save_data_interval) < 0.001) {
                                sim.timestep = std::round(sim.timestep / 0.5) * 0.5;
                                // save agent-level data
                                sim.save_data(i);

                                // save timing data
                                auto cur_time = std::chrono::high_resolution_clock::now();
                                planner_file << sp.num_agents << std::string(",")
                                << p << std::string(",") // periodic or not
                                << i << std::string(",") // trial id
                                << sim.timestep << std::string(",")
                                << sim.planner->search_call_count << std::string(",")
                                << sim.planner->is_invalid_step_call_count << std::string(",")
                                << sim.planner->replan_count << std::string(",")
                                << std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - trial_start_time).count() << std::endl;
                            }
                    
                            sim.update();
                        }
                    
                        if (!sp.outfile_name.empty()) { sim.save_data(i); }
                    }

                    auto trial_end_time = std::chrono::high_resolution_clock::now();

                    // end of trial: want to save info from planner
                    planner_file << sp.num_agents << std::string(",")
                    << p << std::string(",") // periodic or not
                    << i << std::string(",") // trial id
                    << sim.timestep << std::string(",")
                    << sim.planner->search_call_count << std::string(",")
                    << sim.planner->is_invalid_step_call_count << std::string(",")
                    << sim.planner->replan_count << std::string(",")
                    << std::chrono::duration_cast<std::chrono::milliseconds>(trial_end_time - trial_start_time).count() << std::endl;
                }

                sim.outfile.close();

                auto this_end_time = std::chrono::high_resolution_clock::now();
                auto this_duration = std::chrono::duration_cast<std::chrono::milliseconds>(this_end_time - this_start_time);

                complete += 1;
            printf("\x1B[36mJust ran World %i / %i in %lli milliseconds: periodic %i, robots %i \n\033[0m\t\t", complete, total_worlds, this_duration.count(), p, num);
        }
    }
    auto all_end_time = std::chrono::high_resolution_clock::now();
    auto all_duration = std::chrono::duration_cast<std::chrono::milliseconds>(all_end_time - all_start_time);
    std::cout << "\nTime taken to run all trials: " << all_duration.count() << " milliseconds" << std::endl;

    // close outfiles
    planner_file.close();

}