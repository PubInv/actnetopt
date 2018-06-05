#ifndef PLAYGROUND_H
#define PLAYGROUND_H 1

#include "Tetrahelix.hpp"
// #include "Obstacle.hcpp"

#include <SDL.h>


Tetrahelix *init_Tetrahelix(int truss_nodes,double upper, double lower, double initial);

void render_all(SDL_Renderer* renderer, Tetrahelix *an,column_vector* coordsx,Obstacle obstacle);

extern const int WIN_WIDTH;
extern const int WIN_HEIGHT;
// extern Obstacle obstacle;
extern double* best_distances;
extern double best_score;


#define TRUSS_NODES 6
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5
#define CRISIS_DERIVATIVE_LEVEL 1000.0


int mainx(Tetrahelix *an,column_vector* coords);

class Input
{
// Methods
public:
  void (* buttondown)();
  
  Input(void(*f)());
   ~Input();

   // Call this before any other methods
   void readInput();

   bool* getInput();

   // Check this each frame
   bool windowClosed();

  bool mouseDown;
  SDL_Keycode sdl_keycode;

  double x;
  double y;
  
// Data
private:
   SDL_Event m_event;
   bool m_keysHeld[323]; 
   bool m_windowClosed;
};


void mousedown_function();
void handle_goal_target(Tetrahelix *an,column_vector* coordsx,int x,int y);
void handle_goal_target_physical(Tetrahelix *an,column_vector* coordsx,double x,double y,double z);
void handle_goal_target(Tetrahelix *an,column_vector* coordsx,column_vector gl);
void init_renderer(Tetrahelix *an,column_vector* coordsx,Obstacle obstacle);
void close_renderer();
void render_all(Tetrahelix *an,column_vector* coordsx,Obstacle obstacle);

void solve_inverse_problem(Tetrahelix *an);

#endif
