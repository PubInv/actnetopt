#ifndef PLAYGROUND_H
#define PLAYGROUND_H 1

#include "TriLadder.h"

TriLadder *init_TriLadder();
void render_all(SDL_Renderer* renderer, TriLadder *an,column_vector* coordsx,Obstacle obstacle);

extern const int WIN_WIDTH;
extern const int WIN_HEIGHT;
extern Obstacle obstacle;
extern double* best_distances;
extern double best_score;

int mainx(TriLadder *an,column_vector* coords,Obstacle ob);

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
void handle_goal_target(TriLadder *an,column_vector* coordsx,int x,int y);
void init_renderer(TriLadder *an,column_vector* coordsx,Obstacle obstacle);
void close_renderer();

#endif
