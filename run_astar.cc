#include "astar_utils.hh"
#include "astar_manager.hh"
#include "astar_canvas.hh"

int main(int argc, char* argv[])
{
    printf("running run_astar.cc\n");

    sim_params sp;

    sp.num_agents = 2;
    sp.periodic = true;
    sp.diags = false;
    sp.r_upper = 8;

    sp.cells_per_side = 10;

    sp.dt = .1;

    sp.gui_speedup = 0.25; // speed up gui compared to real time
    // sp.gui_draw_every = 5; // update gui every x updates
    sp.gui_zoom = 20; // zoom in on gui
    sp.gui_draw_cells = true;
    sp.gui_draw_footprints = true;
    sp.gui_random_colors = true;

    // 
    Pose pose(0,0,0,0);

    AStarManager sim = AStarManager(sp);

    printf("Try opening a GUI window...\n");
    // sim.reset();

    Fl_Window win(700, 700, "A* Pathfinding");
    Canvas gui = Canvas(&sim, 0,0, win.w(), win.h());

    win.resizable(&gui);

    // win.end();
    win.show();
    gui.startAnimation();
    return Fl::run();
}