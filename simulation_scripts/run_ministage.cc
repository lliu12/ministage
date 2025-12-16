// Run and display simulation

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
    sp.num_agents = 4;
    sp.periodic = true;
    sp.circle_arena = false;
    sp.r_upper = 5;
    sp.r_lower = 0;
    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 0.6;
    sp.cells_range = 2 * 4;
    if(sp.periodic) { sp.cells_range = sp.r_upper; }
    sp.use_sorted_agents = false;
    sp.use_cell_lists = true;
    sp.anglenoise = 1.0;
    sp.anglebias = 0;
    sp.noise_prob = 1;
    sp.conditional_noise = false; 
    sp.avg_runsteps = 10;
    sp.randomize_runsteps = true;
    sp.cruisespeed = 0.5;
    sp.turnspeed = -1; 
    sp.dt = .1;
    sp.goal_tolerance = 0.6;

    sp.gui_random_colors = false; // make all robots gray vs. give robots and goals individual matching colors

    sp.gui_speedup = 3; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 50; // zoom in on gui
    sp.gui_draw_cells = false;
    sp.gui_draw_footprints = true;
    

    sp.outfile_name = "";
    sp.save_data_interval = 10.0;

    sp.verbose = false;

    SimulationManager sim = SimulationManager(sp);

    if (1) {   
        // try opening a GUI window
        printf("Try opening a GUI window...\n");
        sim.reset();

        Fl_Window win(700, 700, "MiniStage");
        Canvas gui = Canvas(&sim, 0,0, win.w(), win.h());

        win.resizable(&gui);

        // win.end();
        win.show();
        gui.startAnimation();
        return Fl::run();
    }


    return 0;

}