#include "canvas.hh"

Canvas::Canvas(SimulationManager *simulation, int x, int y, int width, int height)
    : Fl_Gl_Window(x, y, width, height), sim(simulation)
{
    _scale = sim->sp.gui_zoom;
    _x = 0;
    _y = 0;
    paused = false;
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
    for (Agent *a : sim->agents) {
        a->draw();
    }

    // Draw cells
    if (sim->sp.use_cell_lists & sim->sp.gui_draw_cells) {
        for (const auto& row : sim->sd->cells) {
            for (Cell *c : row) {
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
        // // keep resetting simulation (to test reset functions)
        // if (sim->sd->sim_time > 60) {
        //     sim->reset();
        // }
    }

    // // // Swap buffers to display the rendered content
    // glFlush();
    // swap_buffers();

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
            }
            return 1;
    }

    return Fl_Gl_Window::handle(event);
}


// void Canvas::move(double x, double y)
// {
//     // convert screen points into world points
//     x = x / (_scale);
//     y = y / (_scale);

//     // adjust for pitch angle
//     y = y / cos(dtor(_pitch));

//     // don't allow huge values
//     if (y > 100)
//     y = 100;
//     else if (y < -100)
//     y = -100;

//     // adjust for yaw angle
//     double yaw = -dtor(_yaw);
//     _x += cos(yaw) * x;
//     _y += -sin(yaw) * x;

//     _x += sin(yaw) * y;
//     _y += cos(yaw) * y;

//     glOrtho2d(-_pixels_width / 2.0 / _scale, _pixels_width / 2.0 / _scale,
//             -_pixels_height / 2.0 / _scale, _pixels_height / 2.0 / _scale, _y_min * _scale * 2,
//             _y_max * _scale * 2);
// }


// void Canvas::scale(double scale, double shift_x, double w, double shift_y, double h) {
//     double to_scale = -scale;
//     const double old_scale = _scale;

//     // TODO setting up the factor can use some work
//     double factor = 1.0 + fabs(to_scale) / 25;
//     if (factor < 1.1)
//         factor = 1.1; // this must be greater than 1.
//     else if (factor > 2.5)
//         factor = 2.5;

//     // convert the shift distance to the range [-0.5, 0.5]
//     shift_x = shift_x / w - 0.5;
//     shift_y = shift_y / h - 0.5;

//     // adjust the shift values based on the factor (this represents how much the
//     // positions grows/shrinks)
//     shift_x *= factor - 1.0;
//     shift_y *= factor - 1.0;

//     if (to_scale > 0) {
//         // zoom in
//         _scale *= factor;
//         move(shift_x * w, -shift_y * h);
//     } else {
//         // zoom out
//         _scale /= factor;
//         if (_scale < 1) {
//         _scale = 1;
//         } else {
//         // shift camera to follow where mouse zoomed out
//         move(-shift_x * w / old_scale * _scale, shift_y * h / old_scale * _scale);
//         }
//     }
// }