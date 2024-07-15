#include "agents.hh"

// FLTK Gui includes
#include <FL/fl_draw.H>
#include <FL/gl.h> // FLTK takes care of platform-specific GL stuff
// except GLU
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif


///////////////////////////////////////////////////////////////////////////
// Define Agent class functions

// Constructor
Agent::Agent(int agent_id, sim_params *sim_params, SimulationData *sim_data) {
    sp = sim_params;
    sd = sim_data;    
    id = agent_id;
    cur_pos = new Pose();
    color = Color::RandomColor();
    reset();
}
Agent::Agent() {
}

// Destructor
Agent::~Agent(void){ delete cur_pos;}

// Use rejection sampling to obtain a random point in a the ring between radius r_lower and r_upper (center at origin)
// Or, if not in a circular arena, in the square with center at origin and side length 2 * r_upper
Pose Agent::random_pos()
{
    bool done = 0;
    double rand_x;
    double rand_y;
    double rand_a = 2 * M_PI * (Random::get_unif_double(0, 1) - .5);

    while (!done) {
        rand_x = sp->r_upper * 2 * (Random::get_unif_double(0, 1) - .5);
        rand_y = sp->r_upper * 2 * (Random::get_unif_double(0, 1) - .5);
        double dist = Pose(rand_x, rand_y, 0, 0).Distance(Pose(0,0,0,0));
        if (!sp->circle_arena || (dist <= sp->r_upper && dist >= sp->r_lower)) { done = 1; }
    }

    if (sp->verbose) {
    printf("Random Pose (x, y, angle) generated for agent %i is [%.2f %.2f %.2f] \n", id, rand_x, rand_y, rand_a);
    }    

    return Pose(rand_x, rand_y, 0, rand_a);
}

void Agent::reset() {
    fwd_speed = 0;
    turn_speed = 0;
    set_pos(random_pos());
    trail.clear();
    sensed.clear();
}

// Function to set new position
void Agent::set_pos(Pose p) {
    *cur_pos = p;
}

// Function to get Pose
Pose Agent::get_pos() const {
    return *cur_pos;
}

// Update sensor information
void Agent::sensing_update() {

}

// Update the robot's intended forward and turning speed
void Agent::decision_update() {

}

//// Update robot position
void Agent::position_update() {
    // find the change of pose due to our forward and turning motions
    const Pose dp(fwd_speed * sp->dt, 0, 0, normalize(turn_speed * sp->dt));

    // the pose we're trying to achieve
    Pose newpose(*cur_pos + dp);
    set_pos(newpose);

    // update location if world is periodic and robot is now out of bounds
    if (sp->periodic) {
        double s = 2 * sp->r_upper;

        if (cur_pos->x < -s/2 || cur_pos->x > s/2 || cur_pos->y < -s/2 || cur_pos->y > s/2) { // if out of bounds
        double x = fmod(cur_pos->x + s/2, s) - s/2;
        double y = fmod(cur_pos->y + s/2, s) - s/2;
        set_pos(Pose(x > -s/2 ? x : x + s, y > -s/2 ? y : y + s, cur_pos->z, cur_pos->a));
        }
    }
    
    if(sp->gui_draw_footprints & (fmod(sd->sim_time, 0.5) <= 0.0001)) {
        update_trail();
    }
}

// update trail storing previous positions
void Agent::update_trail() {
    trail.push_back(get_pos());
    if(trail.size() > 40) {
        trail.pop_front();
    }
}

// draw
void Agent::draw() {
    glPushMatrix(); // enter local agent coordinates
    pose_shift(get_pos());

        // draw disk at robot position
        glColor4f(.5, .5, .5, .8); // gray
        GLUquadric *robot_pos = gluNewQuadric();
        gluQuadricDrawStyle(robot_pos, GLU_FILL);
        gluDisk(robot_pos, 0, 0.15, 20, 1);
        gluDeleteQuadric(robot_pos);

        // draw wedge for robot FOV
        glColor4f(0, 0, 1, 0.15); // blue
        GLUquadric *fov = gluNewQuadric();
        gluQuadricDrawStyle(fov, GLU_FILL);
        gluPartialDisk(fov, 0, sp->sensing_range, 20, 1,
                    rtod(M_PI / 2.0 + sp->sensing_angle / 2.0), // start angle
                    rtod(-sp->sensing_angle)); // sweep angle
        gluDeleteQuadric(fov);

        // // draw heading
        // glBegin(GL_LINES);
        //     glColor4f(0, 0, 1, 1); 
        //     glVertex2f(0,0);              // x, y
        //     glVertex2f(sp->sensing_range, 0);
        // glEnd();

    glPopMatrix();

    // draw trail
    if(sp->gui_draw_footprints) {
        for (Pose p : trail) {
            glPushMatrix();
            pose_shift(p);
                // draw disk at footprint position
                glColor4f(.5, .5, .5, .3); // gray
                GLUquadric *robot_pos = gluNewQuadric();
                gluQuadricDrawStyle(robot_pos, GLU_FILL);
                gluDisk(robot_pos, 0, 0.15, 20, 1);
                gluDeleteQuadric(robot_pos);
            glPopMatrix();
        }
    }
    
}



///////////////////////////////////////////////////////////////////////////
// Define GoalAgent class functions

GoalAgent::GoalAgent(int agent_id, sim_params *sim_params, SimulationData *sim_data) : Agent(agent_id, sim_params, sim_data){

}

GoalAgent::GoalAgent() : Agent() {}

// Destructor
GoalAgent::~GoalAgent(void){}

void GoalAgent::reset() {
    Agent::reset();

    stop = 0;
    goal_pos = random_pos(); // set goal
    goal_birth_time = sd->sim_time;
    goals_reached = 0;
    travel_angle = 0;
}

// make updates when robot reaches goal (increase goal counters, generate new goal, etc)
void GoalAgent::goal_updates() {
    goal_pos = random_pos();
    goals_reached++;
    goal_birth_time = sd->sim_time;
}

// Update sensor information
void GoalAgent::sensing_update() {

    // first, check if robot has reached its goal and update variables accordingly
    if (cur_pos->Distance(goal_pos) < sp->goal_tolerance) {
        goal_updates();
    }

    sensed = sd->sense(id, get_pos());
    stop = sensed.size() > 0; // agent will stop if any neighbor was sensed in vision cone

    decision_update();
}


// Update the robot's intended forward and turning speed
void GoalAgent::decision_update() {
    travel_angle = angle_to_goal();
    
    fwd_speed = stop ? 0 : sp->cruisespeed;

    // for instantaneous turning, set robot to travel angle
    if (sp->turnspeed == -1) {
      set_pos(Pose(cur_pos->x, cur_pos->y, cur_pos->z, travel_angle));
      turn_speed = 0;
    }
    // for non-instantaneous turning, set turnspeed
    else {
        double a_error = normalize(travel_angle - cur_pos->a);
        turn_speed = sp->turnspeed * a_error;
    }
}


//// Get (global) angle robot should move in to head straight to goal
double GoalAgent::angle_to_goal() {
      Pose goal_pos_helper; // will be true goal pos if world is not periodic
      if (!sp->periodic) {
        goal_pos_helper = goal_pos;
      }

      // if space is periodic, figure out where robot should move to for shortest path to goal
      else {
            goal_pos_helper = nearest_periodic(get_pos(), goal_pos, sp->r_upper);
      }
      
      double x_error = goal_pos_helper.x - cur_pos->x;
      double y_error = goal_pos_helper.y - cur_pos->y;

      return atan2(y_error, x_error);
}

// Draw goals
void GoalAgent::draw() {
    Agent::draw();

    // draw small point at robot goal
    glPushMatrix(); 
        pose_shift(goal_pos);
            glColor4f(1, 0, .8, .7); // magenta
            GLUquadric *goal = gluNewQuadric();
            gluQuadricDrawStyle(goal, GLU_FILL);
            gluDisk(goal, 0, 0.12, 20, 1);
            gluDeleteQuadric(goal);
    glPopMatrix();
}



///////////////////////////////////////////////////////////////////////////
// Define ConstNoiseAgent class functions


ConstNoiseAgent::ConstNoiseAgent(int agent_id, sim_params *sim_params, SimulationData *sim_data) 
    : GoalAgent(agent_id, sim_params, sim_data) {}

ConstNoiseAgent::ConstNoiseAgent() : GoalAgent() {}

// Destructor
ConstNoiseAgent::~ConstNoiseAgent(void){}

void ConstNoiseAgent::reset() {
    GoalAgent::reset();
    current_phase_count = 0;
}

void ConstNoiseAgent::goal_updates() {
    GoalAgent::goal_updates();
    current_phase_count = 0;
}

// Determine angle for robot to steer in (after adding noise)
double ConstNoiseAgent::get_travel_angle() {
  return angle_to_goal() + (sp->anglenoise == -1 ? Random::get_unif_double(-M_PI, M_PI) : Random::get_normal_double(sp->anglebias, sp->anglenoise));
}

// Update the robot's intended forward and turning speed
void ConstNoiseAgent::decision_update() {
    // check if current run phase is over
    if (current_phase_count >= runsteps) {
        current_phase_count = 0;
    }

    if (current_phase_count == 0) {
        // if a new run phase is beginning, get random runlength between 1/2 and 3/2 of provided runsteps
        if (sp->randomize_runsteps) {
            int lower = std::round(sp->avg_runsteps / 2);
            int higher = std::round(3 * sp->avg_runsteps / 2);
            runsteps = Random::get_unif_int(lower, higher);

        }
        else {runsteps = sp->avg_runsteps;}

        // also get travel angle
        travel_angle = get_travel_angle();

        // for instantaneous turning, set robot to travel angle
        if (sp->turnspeed == -1) {
            Pose cur_pos = get_pos();
            set_pos(Pose(cur_pos.x, cur_pos.y, cur_pos.z, travel_angle));
            turn_speed = 0;
        }
    }

    fwd_speed = (stop ? 0 : sp->cruisespeed);

    // for non-instantaneous turning, set turnspeed
    if (sp->turnspeed != -1) {
        double a_error = normalize(travel_angle - get_pos().a);
        turn_speed = sp->turnspeed * a_error;
    }
    current_phase_count++;
}



///////////////////////////////////////////////////////////////////////////
// Define NoiseAgent class functions


NoiseAgent::NoiseAgent(int agent_id, sim_params *sim_params, SimulationData *sim_data) 
    : ConstNoiseAgent(agent_id, sim_params, sim_data) {}

NoiseAgent::NoiseAgent() : ConstNoiseAgent() {}

// Destructor
NoiseAgent::~NoiseAgent(void){}

// Determine angle for robot to steer in (after adding noise)
double NoiseAgent::get_travel_angle() {
    double without_noise = angle_to_goal();
    double with_noise = ConstNoiseAgent::get_travel_angle();

    if (!(sp->conditional_noise) || stop) { // unless conditional noise is on and robot is free to move,
    // add noise to motion with noise_prob probability
        if (Random::get_unif_double(0, 1) <= sp->noise_prob) {
            return with_noise;
        }
    }

    // otherwise head directly to goal
    return without_noise;
}