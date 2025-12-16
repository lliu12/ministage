#include "astar_utils.hh"
#include "astar_manager.hh"
#include "astar_canvas.hh"

const char* redText = "\033[1;31m";
const char* resetText = "\033[0m";
#ifndef IS_TRUE
#define IS_TRUE(x) { if (!(x)) std::cout << redText << __FUNCTION__ << " FAILED on line " << __LINE__ << resetText << std::endl; }
#endif


int main(int argc, char* argv[])
{
    sim_params sp;

    sp.num_agents = 2;
    sp.periodic = true;
    sp.diags = true;
    sp.r_upper = 12;
    sp.diags_take_longer = true;

    sp.cells_per_side = 15;

    sp.sensing_angle = M_PI * 2.0 / 3.0;
    sp.sensing_range = 2.5;

    sp.time_steps = 3000;

    sp.gui_speedup = sp.diags_take_longer ? 1.25 : 0.75 ; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = true;
    sp.gui_random_colors = true;
    sp.dt = .3; // this is used only for GUI

    sp.verbose = false;

    AStarManager sim = AStarManager(sp);
    
    {
    printf("Try opening a GUI window...\n");
    // sim.reset();

    Fl_Window win(900, 900, "A* Pathfinding");
    Canvas gui = Canvas(&sim, 0,0, win.w(), win.h());

    win.resizable(&gui);

    // win.end();
    win.show();
    gui.startAnimation();
    return Fl::run();
    }
}