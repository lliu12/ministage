#include "ministage.hh"

// FLTK Gui includes
#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Window.H>
#include <FL/fl_draw.H>
#include <FL/gl.h> // FLTK takes care of platform-specific GL stuff
// except GLU
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

class Canvas : public Fl_Gl_Window {
    public:
    Canvas(SimulationManager *simulation, int x, int y, int width, int height);
    ~Canvas();

    bool paused;

    // double _pitch; // left-right (about y)
    // double _yaw; // up-down (about x)
    // double _x, _y, _z;
    // double _scale;
    // double _pixels_width;
    // double _pixels_height;
    // double _y_min;
    // double _y_max;

    SimulationManager *sim;

    void draw() override;
    int handle(int event) override;

    static void TimerCallback(void* userdata);
    void startAnimation();

    // void scale(double scale, double shift_x, double w, double shift_y, double h);
    // void move(double x, double y);


};




