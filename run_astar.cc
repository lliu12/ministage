#include "astar_utils.hh"
#include "astar_manager.hh"
#include "astar_canvas.hh"


int main(int argc, char* argv[])
{
    printf("running run_astar.cc\n");

    sim_params sp;

    sp.num_agents = 2;
    sp.periodic = false;
    sp.diags = true;
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

    std::vector<SiteID> plan = sim.planner->search_2d(SiteID(0,6), SiteID(1,4));
    while (!plan.empty()) {
        SiteID s = plan.back();
        printf("%i, %i \n", s.idx, s.idy);
        // std::cout << plan.back() << std::endl;
        plan.pop_back();
    }


    std::cout << "Checking diagonal distance heuristic function..." << std::endl;
    {
        if (sp.diags) {
            IS_TRUE(sim.planner->dist_heuristic(SiteID(0,2), SiteID(5,0)) == 1.5 * 2 + 3);
            // std::cout << "should be 6 but we get " << sim.planner->dist_heuristic(SiteID(0,2), SiteID(5,0)) << std::endl;
            IS_TRUE(sim.planner->dist_heuristic(SiteID(0,5), SiteID(3,0)) == 1.5 * 3 + 2);
        }
        else {
            IS_TRUE(sim.planner->dist_heuristic(SiteID(0,2), SiteID(5,0)) == 7);
        }
        
    }


    // std::cout << "Testing set sorting for A* search..." << std::endl;
    // {
    //     struct node {
    //         // f = g + h
    //         double f, g, h;
    //         SiteID pos;
    //         SiteID parent;
    //     };

    //     node n1;
    //     n1.f = 0;
    //     n1.g = 10;
    //     n1.h = 0;

    //     node n2; 
    //     n2.f = 1;
    //     n2.g = 9;
    //     n2.h = 1;

    //     node n3;
    //     n3.f = 3;
    //     n3.g = 8;
    //     n3.h = 3;

    //     auto cmp = [](node a, node b) { return a.f < b.f; };
    //     std::set<node, decltype(cmp)> node_set(cmp);
    //     node_set.insert(n3);
    //     node_set.insert(n2);
    //     node_set.insert(n1);
    //     node_set.insert(n2);

    //     for (node nn : node_set) {
    //         std::cout << nn.f << std::endl;
    //     }

    // }

    {
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
}