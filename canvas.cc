#include "canvas.hh"

Canvas::Canvas(SimulationManager sim, int x, int y, int width, int height)
    : Fl_Gl_Window(x, y, width, height) {

}

Canvas::~Canvas() {

}

void Canvas::draw() {

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // Set background color to black and opaque
    glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer (background)


//     glColor3f(0.0f, .5f, 0.0f);
//     // An array of 3 vectors which represents 3 vertices
//     static const GLfloat g_vertex_buffer_data[] = {
//     -0.5f, -1.0f, 0.0f,
//     1.0f, -1.0f, 0.0f,
//     0.0f,  1.0f, 0.0f,
//     };


//     // This will identify our vertex buffer
//     GLuint vertexbuffer;
//     // Generate 1 buffer, put the resulting identifier in vertexbuffer
//     glGenBuffers(1, &vertexbuffer);
//     // The following commands will talk about our 'vertexbuffer' buffer
//     glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//     // Give our vertices to OpenGL.
//     glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);


//     // 1st attribute buffer : vertices
//     glEnableVertexAttribArray(0);
//     glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
//     glVertexAttribPointer(
//     0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
//     3,                  // size
//     GL_FLOAT,           // type
//     GL_FALSE,           // normalized?
//     0,                  // stride
//     (void*)0            // array buffer offset
//     );
//     // Draw the triangle !
//     glDrawArrays(GL_TRIANGLES, 0, 3); // Starting from vertex 0; 3 vertices total -> 1 triangle
//     glDisableVertexAttribArray(0);



    glBegin(GL_QUADS);              // Each set of 4 vertices form a quad
      glColor3f(0.0f, 1.0f, 1.0f); // Red
      glVertex2f(-0.1f, -0.1f);    // x, y
      glVertex2f( 0.1f + .2 * x_, -0.1f);
      glVertex2f( 0.1f,  0.1f);
      glVertex2f(-0.1f,  0.1f);
   glEnd();

    glBegin(GL_QUADS);              // Each set of 4 vertices form a quad
      glColor3f(0.0f, 0.0f, 1.0f); // Red
      glVertex2f(0, .1);    // x, y
      glVertex2f(.1 * x_, 0);
      glVertex2f(.1,  0);
      glVertex2f(.1,  .1);
   glEnd();

    // Draw a rectangle
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POLYGON);
    glVertex2f(x_, y_);
    glVertex2f(x_ + 5, y_);
    glVertex2f(x_ + 5, y_ + 30);
    glVertex2f(x_, y_ + 30);
    glEnd();


    // setup code
    static GLUquadricObj *q;
    q = gluNewQuadric();
    gluQuadricNormals (q,GLU_TRUE);
    gluQuadricTexture (q,GLU_TRUE);

    // the gluDisk based shapes
    gluPartialDisk (q, 0.6, 0.8, 2, 1,0, 240); 
    // gluPartialDisk (q, 0.0, 0.8, 30, 1, -45, 270); // FOV



    // Move the rectangle horizontally
    x_ += 1;

    // Reset position when reaching the right edge
    if (fabs(x_) > w())
        x_ = 0;

    // Swap buffers to display the rendered content
    glFlush();
    swap_buffers();

    // Schedule a redraw to create the animation loop
    redraw();


}

void Canvas::TimerCallback(void* userdata) {
    Canvas* animation = (Canvas*)userdata;
    animation->redraw();
    Fl::repeat_timeout(0.5, TimerCallback, userdata);  // Adjust the interval as needed
}

void Canvas::startAnimation() {
    Fl::add_timeout(0.5, TimerCallback, this);  // Start the animation loop
}

int Canvas::handle(int event) {
    return Fl_Gl_Window::handle(event);
}