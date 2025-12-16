// Running this script produces the local navigation methods data used in 
// Main Text Fig. 4 of "Noise-Enabled Goal Attainment in Crowded Collectives"

#include <chrono>
#include <filesystem>
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
    std::vector<int> num_agents_arr = {1, 16, 32, 64, 96, 128, 160, 192, 224, 256};

    // noise levels to try with Gaussian noise
    std::vector<float> gaussian_noise_arr = {0, 0.5, 0.6, 0.7, 0.8, 0.9, 1., 1.1, 1.2, 1.3, 1.4, 1.5}; // run these all with conditional noise off, anglenoise varies, noise prob = 1
    
    sp.anglebias = 0;

    // initialize output files
    std::filesystem::path base_dir = SIM_DATA_DIR;
    std::filesystem::create_directories(base_dir);
    std::string projectname = (base_dir / "fig4_local").string();
    std::string trials_filename = projectname + "_trials_data.txt"; // save per-trial info about planner function calls
    sp.outfile_name = projectname + "_agents_data.txt"; // save info about agents and goal attainment

    // output file headings
    std::ofstream trials_file_head(trials_filename, std::ios::out);
    trials_file_head << "num_robots,noise,periodic,trial,sim_time,noise_type,sensing_call_count,runtime_ms\n";
    trials_file_head.close();

    std::ofstream agents_file_head(sp.outfile_name, std::ios::out);
    agents_file_head << "trial,periodic,num_robots,noise,noise_prob,sim_time,robot_id,x_pos,y_pos,angle,goal_x_pos,goal_y_pos,goal_birth_time,goals_reached,stopped,nearby_robot,addtl_data\n";
    agents_file_head.close();

    std::ofstream trials_file;
    trials_file << std::fixed << std::setprecision(2);
    trials_file.open(trials_filename, std::ios_base::app);


    sp.save_data_interval = 1000.;
    sp.turnspeed = -1;
    sp.circle_arena = false;
    int num_trials = 20;


    // parameters to leave unchanged
    sp.r_upper = 20;
    sp.r_lower = 0;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 2.;

    sp.cells_range = 20;
    sp.cells_per_side = floor(2.0 * sp.cells_range / sp.sensing_range);
    sp.cell_width = 2.0 * sp.cells_range / sp.cells_per_side;
    sp.use_sorted_agents = false;
    sp.use_cell_lists = true;
    

    sp.avg_runsteps = 50;
    sp.randomize_runsteps = true;

    sp.cruisespeed = 0.5;
    sp.goal_tolerance = 0.6;

    sp.dt = .1;

    sp.gui_speedup = 25; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = false;
    sp.verbose = false;

    IS_TRUE(2 * sp.cells_range / sp.cells_per_side >= sp.sensing_range);




    // int total_worlds = periodic_arr.size() * num_agents_arr.size() * (gaussian_noise_arr.size() + noise_prob_arr.size() + 1);
    int total_worlds = periodic_arr.size() * num_agents_arr.size() * (gaussian_noise_arr.size() + 1);
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

                // in-planner outfile for saving data about agent positions
                sim.outfile << std::fixed << std::setprecision(2);
                sim.outfile.open(sp.outfile_name, std::ios_base::app);

                for (int i = 0; i < num_trials; i++) {
                    
                    auto trial_start_time = std::chrono::high_resolution_clock::now();
                    {
                        sim.reset();
                        while (sim.sd->sim_time < sim_run_length) {
                    
                            if (!sp.outfile_name.empty() && fmod(sim.sd->sim_time, sp.save_data_interval) < 0.001) {
                                sim.save_data(i);

                                // save timing data
                                auto cur_time = std::chrono::high_resolution_clock::now();
                                trials_file << sp.num_agents << std::string(",")
                                << sp.anglenoise << std::string(",")
                                << p << std::string(",") // periodic or not
                                << i << std::string(",") // trial id
                                << sim.sd->sim_time << std::string(",")
                                << sp.addtl_data << std::string(",")
                                << sp.num_agents * sim.sd->sim_time / sp.dt << std::string(",")
                                << std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - trial_start_time).count() << std::endl;
                        
                            }

                            sim.update();
                        }
                    
                        if (!sp.outfile_name.empty()) { sim.save_data(i); }
                    }


                    auto trial_end_time = std::chrono::high_resolution_clock::now();

                    // end of trial: want to save info from planner
                    trials_file << sp.num_agents << std::string(",")
                    << sp.anglenoise << std::string(",")
                    << p << std::string(",") // periodic or not
                    << i << std::string(",") // trial id
                    << sim.sd->sim_time << std::string(",")
                    << sp.addtl_data << std::string(",")
                    << sp.num_agents * sim.sd->sim_time / sp.dt << std::string(",")
                    << std::chrono::duration_cast<std::chrono::milliseconds>(trial_end_time - trial_start_time).count() << std::endl;
                }
                sim.outfile.close();

                auto this_end_time = std::chrono::high_resolution_clock::now();
                auto this_duration = std::chrono::duration_cast<std::chrono::milliseconds>(this_end_time - this_start_time);

                complete += 1;
                printf("Just ran World %i / %i in %lli milliseconds: periodic %i, robots %i, noise %f \n", complete, total_worlds, this_duration.count(), p, num, noise);
                
            }
        }
    }


    printf("Running conditional noise worlds...");
    for (bool p : periodic_arr) {
        for (int num : num_agents_arr) {
            sp.periodic = p;
            sp.num_agents = num;
            sp.anglenoise = -1; // -1 for uniform random noise
            sp.noise_prob = 1;
            sp.conditional_noise = true;
            sp.addtl_data = "conditional noise";

            auto this_start_time = std::chrono::high_resolution_clock::now();

            SimulationManager sim = SimulationManager(sp);

            sim.outfile << std::fixed << std::setprecision(2);
            sim.outfile.open(sp.outfile_name, std::ios_base::app);

            for (int i = 0; i < num_trials; i++) {
                
                auto trial_start_time = std::chrono::high_resolution_clock::now();
                // sim.run_trial(sim_run_length, i);
                {
                    sim.reset();
                    while (sim.sd->sim_time < sim_run_length) {
                
                        if (!sp.outfile_name.empty() && fmod(sim.sd->sim_time, sp.save_data_interval) < 0.001) {
                            sim.save_data(i);

                            // save timing data
                            auto cur_time = std::chrono::high_resolution_clock::now();
                            trials_file << sp.num_agents << std::string(",")
                            << sp.anglenoise << std::string(",")
                            << p << std::string(",") // periodic or not
                            << i << std::string(",") // trial id
                            << sim.sd->sim_time << std::string(",")
                            << sp.addtl_data << std::string(",")
                            << sp.num_agents * sim.sd->sim_time / sp.dt << std::string(",")
                            << std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - trial_start_time).count() << std::endl;
                    
                        }

                        sim.update();
                    }
                
                    if (!sp.outfile_name.empty()) { sim.save_data(i); }
                }

                auto trial_end_time = std::chrono::high_resolution_clock::now();

                // end of trial: want to save info from planner
                trials_file << sp.num_agents << std::string(",")
                << sp.anglenoise << std::string(",")
                << p << std::string(",") // periodic or not
                << i << std::string(",") // trial id
                << sim.sd->sim_time << std::string(",")
                << sp.addtl_data << std::string(",")
                << sp.num_agents * sim.sd->sim_time / sp.dt << std::string(",")
                << std::chrono::duration_cast<std::chrono::milliseconds>(trial_end_time - trial_start_time).count() << std::endl;
            }
            sim.outfile.close();

            auto this_end_time = std::chrono::high_resolution_clock::now();
            auto this_duration = std::chrono::duration_cast<std::chrono::milliseconds>(this_end_time - this_start_time);

            complete += 1;
            printf("Just ran World %i / %i in %lli milliseconds: periodic %i, robots %i, conditional noise \n", complete, total_worlds, this_duration.count(), p, num);

        }
    }

    auto all_end_time = std::chrono::high_resolution_clock::now();
    auto all_duration = std::chrono::duration_cast<std::chrono::milliseconds>(all_end_time - all_start_time);
    std::cout << "\nTime taken to run all trials: " << all_duration.count() << " milliseconds" << std::endl;

    trials_file.close();

}