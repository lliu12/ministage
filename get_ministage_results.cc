#include <chrono>
#include "simulation_manager.hh"
#include "canvas.hh"

const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }


int main(int argc, char* argv[])
{

    sim_params sp;

    // bool periodic_arr[2] = [false, true];
    // int num_agents_arr[16] = [16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 256];
    // int noise_arr[9] = [-1, 0, .25, .5, .75, 1, 1.5, 1.75, 2.0]

    
    sp.periodic = false;
    sp.num_agents = 200;
    sp.anglenoise = -1; // -1 for uniform random noise, otherwise this is the STD of gaussian noise 
    sp.anglebias = 0;
    sp.turnspeed = -1; // -1 for instant turning, 10 is a good value otherwise for visuals


    // parameters to leave unchanged
    sp.circle_arena = false;
    sp.r_upper = 8;
    sp.r_lower = 0;

    sp.sensing_angle = M_PI * 3.0 / 4.0;
    sp.sensing_range = 0.6;

    sp.cells_range = 25;
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

    sp.outfile_name = "test_saving_ministage_results.txt";
    sp.save_data_interval = 10.0;

    IS_TRUE(2 * sp.cells_range / sp.cells_per_side >= sp.sensing_range);

    

    // make and run a lot of worlds to check for memory leaks

    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1; i++)
    {
    if (i % 100 == 0) { printf("at world %i \n", i);}

    SimulationManager sim = SimulationManager(sp);
    auto start_time = std::chrono::high_resolution_clock::now();
    sim.reset();
    sim.run_trials(5, 100);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    // std::cout << "in main(): sp1.use_count() == " << sim.sd.use_count()
    //           << " (object @ " << sim.sd << ")\n";

    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    std::cout << "Time taken to run: " << duration.count() << " seconds" << std::endl;


    // // try running a trial
    // {
    // sp.use_sorted_agents = false;
    // sp.use_cell_lists = true;
    // SimulationManager sim = SimulationManager(sp);
    // printf("Try running a trial...\n");
    // auto start_time = std::chrono::high_resolution_clock::now();
    // sim.reset();
    // sim.run_trials(10, 1200);
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    // std::cout << "Time taken to run with cell lists: " << duration.count() << " seconds" << std::endl;
    // }


    // // try running a trial
    // {
    // sp.use_sorted_agents = true;
    // sp.use_cell_lists = false;
    // SimulationManager sim = SimulationManager(sp);
    // printf("Try running a trial...\n");
    // auto start_time = std::chrono::high_resolution_clock::now();
    // sim.reset();
    // sim.run_trials(10, 1200);
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    // std::cout << "Time taken to run with sorted lists: " << duration.count() << " seconds" << std::endl;
    // }

}