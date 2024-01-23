#include "canvas.hh"

Canvas::Canvas(SimulationManager simulation, int x, int y, int width, int height)
    : Fl_Gl_Window(x, y, width, height), sim(simulation)
{
    x_test = 0;
}

Canvas::~Canvas() {

}

void Canvas::draw() {
    glLoadIdentity();
    glViewport(0, 0, w(), h()); // 0,0 in bottom left corner

    gluOrtho2D(0.0, w(), 0.0, h()); // set coordinate system with origin in lower left corner
    glTranslatef(w() / 2, h() / 2, 0); // translate origin to center
    glScalef(5, 5, 1.0f); // zoom in a bit

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

    // glPushMatrix();
    //     glTranslatef(-.5, 0, 0); // okay, this worked! moves it -.5 of screen width. need to add push, translate, and pop. 
        
    //     // setup code
    //     glColor4f(1.0f, 0.0f, 0.0f, 0.2f);
    //     static GLUquadricObj *q;
    //     q = gluNewQuadric();
    //     gluQuadricNormals (q,GLU_TRUE);
    //     gluQuadricTexture (q,GLU_TRUE);

    //     // the gluDisk based shapes
    //     // gluPartialDisk (q, 60, 80, 2, 1,0, 240); 
    //     gluPartialDisk (q, 0.6, 0.8, 2, 1,0, 240); 
    //     // gluPartialDisk (q, 0.0, 0.8, 30, 1, -45, 270); // FOV
    // glPopMatrix();

    // Move the rectangle horizontally
    x_test += 1;

    // Draw robots
    for (int i = 0; i < sim.sp.num_agents; i++) {
        // Pose agent_pose = sim.agents[i]->get_pos();
        glPushMatrix(); // enter local agent coordinates
        pose_shift(sim.agents[i]->get_pos());

            // draw disk at robot position
            glColor4f(.5, .5, .5, .8); // gray
            GLUquadric *robot_pos = gluNewQuadric();
            gluQuadricDrawStyle(robot_pos, GLU_FILL);
            gluDisk(robot_pos, 0, 1, 20, 1);
            gluDeleteQuadric(robot_pos);

            // draw wedge for robot FOV
            glColor4f(0, 0, 1, 0.3); // blue
            GLUquadric *fov = gluNewQuadric();
            gluQuadricDrawStyle(fov, GLU_FILL);
            // gluPartialDisk (quadric, 0.6, 5, 2, 1,0, 0); 
            
            gluPartialDisk(fov, 0, sim.sp.sensing_range,
                        20, // slices
                        1, // loops
                        rtod(M_PI / 2.0 + sim.sp.sensing_angle / 2.0), // start angle
                        rtod(-sim.sp.sensing_angle)); // sweep angle
            gluDeleteQuadric(fov);
        glPopMatrix();



        glPushMatrix(); // enter local agent coordinates
        pose_shift(sim.agents[i]->goal_pos);
            // draw small point at robot goal
            glColor4f(1, 0, 1, .8); // magenta
            GLUquadric *goal = gluNewQuadric();
            gluQuadricDrawStyle(goal, GLU_FILL);
            gluDisk(goal, 0, 0.4, 20, 1);
            gluDeleteQuadric(goal);
        glPopMatrix();

        // update simulation
        sim.update();
    }


    // Draw a rectangle
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POLYGON);
    glVertex2f(x_test, y_test + 15);
    glVertex2f(x_test + 5, y_test + 15);
    glVertex2f(x_test + 5, y_test + 15 + 10);
    glVertex2f(x_test, y_test + 15 + 10);
    glEnd();

    // // // Swap buffers to display the rendered content
    // glFlush();
    // swap_buffers();

    // Schedule a redraw to create the animation loop
    redraw();

}

void Canvas::TimerCallback(void* userdata) {
    Canvas* this_canvas = (Canvas*)userdata;
    this_canvas->redraw();
    Fl::repeat_timeout(this_canvas->sim.sp.dt / this_canvas->sim.sp.gui_speedup, TimerCallback, userdata);  // Adjust the interval as needed
}

void Canvas::startAnimation() {
    Fl::add_timeout(sim.sp.dt / sim.sp.gui_speedup, TimerCallback, this);  // Start the animation loop
}

int Canvas::handle(int event) {
    // switch(event) {
    //     case FL_MOUSEWHEEL:
    //         // scale(Fl::event_dy(), Fl::event_x(), w(), Fl::event_y(), h());
    //         // // invalidate();
    //         // redraw();
    //         return 1;
    // }

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