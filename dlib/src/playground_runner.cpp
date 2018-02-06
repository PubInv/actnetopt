#include <SDL.h>
#include "playground.hpp"
#include "Obstacle.hpp"
#include <stdio.h>
#include <iostream>

using namespace std;


int main( int argc, char* args[] )
{
  TriLadder *an = init_TriLadder();
  
  column_vector* coordsx = new column_vector[an->num_nodes];

  mainx(an,coordsx,obstacle);


    // Setup renderer
    init_renderer(an,coordsx,obstacle);
    // SDL_Renderer* renderer = NULL;
    // renderer =  SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED);

    // render_all(renderer,an,coordsx,obstacle);

    Input input(&mousedown_function);
    int n = 0;
    bool running = true;
    while(running) {
      input.readInput();
      if (input.sdl_keycode == SDLK_q) {
  	cout << "shutdown";
  	running = false;
      }
      if (input.mouseDown) {
	handle_goal_target(an,coordsx,input.x,input.y);
	render_all(an,coordsx,obstacle);
	input.mouseDown = false;	
      }
      SDL_Delay( 10 );
      n++;
    }
    SDL_Event key_event;    
    SDL_WaitEvent(&key_event);
    cout << "Exit key " << key_event.type << "\n";

    //    SDL_DestroyWindow(window);
    //    SDL_Quit();
    close_renderer();    
    return EXIT_SUCCESS;
}
