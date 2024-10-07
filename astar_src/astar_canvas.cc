#include "astar_canvas.hh"

Canvas::Canvas(AStarManager *simulation, int x, int y, int width, int height)
    : Fl_Gl_Window(x, y, width, height), sim(simulation)
{
    _scale = sim->sp.gui_zoom;
    _x = 0;
    _y = 0;
    paused = true;
}

Canvas::~Canvas() {

}

void Canvas::draw() {
    glLoadIdentity();
    glViewport(0, 0, w(), h()); // 0,0 in bottom left corner

    gluOrtho2D(0.0, w(), 0.0, h()); // set coordinate system with origin in lower left corner
    glTranslatef(w() / 2, h() / 2, 0); // translate origin to center
    glScalef(sim->sp.gui_zoom, sim->sp.gui_zoom, 1.0f); // zoom in a bit

    // GL settings
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
    glDepthMask(GL_TRUE);
    glEnable(GL_TEXTURE_2D);
    glEnableClientState(GL_VERTEX_ARRAY);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // White bg
    glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)

    // Draw robots
    for (AStarAgent *a : sim->agents) {
        a->draw();
    }

    // Draw cells
    if (sim->sp.gui_draw_cells) {
        for (const auto& row : sim->space->cells) {
            for (SpaceUnit *c : row) {
                c->draw();
            }
        }
    }

    // Draw boundaries of periodic arena
    if (sim->sp.periodic) {
        glBegin(GL_LINE_LOOP);               // Draw outline of cell, with no fill
            glColor4f(1, 0.7, 0.2, 1);    // Orange outline 
            glVertex2f(-sim->sp.r_upper, -sim->sp.r_upper);              // x, y
            glVertex2f(sim->sp.r_upper, -sim->sp.r_upper);
            glVertex2f(sim->sp.r_upper, sim->sp.r_upper);
            glVertex2f(-sim->sp.r_upper, sim->sp.r_upper);
        glEnd();
    }


    // update simulation
    if (!paused) { 
        sim->update(); 
        // keep resetting simulation (to test reset functions)
        // if (*(sim->planner->timestep) > 30) {
        //     sim->reset();
        // }
    }

    if (sim->sp.num_agents ==2 && sim->agents[0]->cur_pos == sim->agents[1]->cur_pos) {
        printf("\033[31merror: first two agents in same position %i, %i\n\033[0m", sim->agents[0]->cur_pos.idx, sim->agents[0]->cur_pos.idy);
    }


    // // draw dummy obstacles
    // // put in some dummy obstacles
    // for (int i = 0; i < 7; i++) {

    //     meters_t xmin = sim->space->cells[i][5]->xmin;
    //     meters_t xmax = sim->space->cells[i][5]->xmax;
    //     meters_t ymin = sim->space->cells[i][5]->ymin;
    //     meters_t ymax = sim->space->cells[i][5]->ymax;

    //     glBegin(GL_LINE_LOOP);               // Draw outline of cell, with no fill
    //     glColor4f(0.0f, 0.0, 1.0f, 0.8);    // different Green outline
        
    //     glVertex2f(xmin, ymin);              // x, y
    //     glVertex2f(xmax, ymin);
    //     glVertex2f(xmax, ymax);
    //     glVertex2f(xmin, ymax);
    //     glEnd();
    // }


    // // draw simulation time
    // char buffer[50];
    // snprintf(buffer, sizeof(buffer), "Time: %.2f", sim->timestep);
    // // Draw the text using FLTK's text drawing function
    // fl_color(FL_BLACK);
    // fl_draw("hello world", sim->space->cells[0][0]->xmin, sim->space->cells[0][0]->ymin);


    redraw();

}

void Canvas::TimerCallback(void* userdata) {
    Canvas* this_canvas = (Canvas*)userdata;
    this_canvas->redraw();
    Fl::repeat_timeout(this_canvas->sim->sp.dt / this_canvas->sim->sp.gui_speedup, TimerCallback, userdata);  // Adjust the interval as needed
}

void Canvas::startAnimation() {
    Fl::add_timeout(sim->sp.dt / sim->sp.gui_speedup, TimerCallback, this);  // Start the animation loop
}

int Canvas::handle(int event) {
    switch(event) {
        case FL_PUSH:
                // Handle mouse click event
                // printf("mouse clicked \n");
                return 1; // Returning 1 means we handled the event

        case FL_MOUSEWHEEL:
            // scale(Fl::event_dy(), Fl::event_x(), w(), Fl::event_y(), h());
            // // invalidate();
            // redraw();
            return 1;

        // these focus events must return 1 to allow keyboard events
        case FL_FOCUS:
            return 1;
        case FL_UNFOCUS:
            return 1;

        case FL_KEYBOARD:
            switch(Fl::event_key()) {
                case 'p':
                    paused = !paused;

                case '.':
                    if (paused) {
                        sim->update();
                        redraw();
                    }
                    return 1;
            }
            return 1;
    }

    return Fl_Gl_Window::handle(event);
}
