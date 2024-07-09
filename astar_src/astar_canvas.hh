#include "astar_manager.hh"

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
    Canvas(AStarManager *simulation, int x, int y, int width, int height);
    ~Canvas();

    bool paused;

    double _x, _y; 
    double _scale;

    AStarManager *sim;

    void draw() override;
    int handle(int event) override;

    static void TimerCallback(void* userdata);
    void startAnimation();

};




