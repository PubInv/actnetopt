#include <SDL.h>
#include "playground.hpp"
#include "Obstacle.hpp"
#include <stdio.h>
#include <iostream>

using namespace std;


const int WIN_WIDTH = 640 * 2;
const int WIN_HEIGHT = WIN_WIDTH;


const double w_physical = (TRUSS_NODES / 10.0 ) * 12.0 ;
const double h_physical = (TRUSS_NODES / 10.0 ) * 12.0 ;

double physical_to_viewport_x(double px) {
    double x = px;
     // first scale appropriately
    x = x * (WIN_WIDTH / (2.0 * w_physical));
     // now move to origin....
    x += WIN_WIDTH/2.0;
    return x;
}
double physical_to_viewport_y(double py) {
    double y = py;
    // first scale appropriately
    y = y * (WIN_HEIGHT / (2.0 * h_physical));
    
    // now move to origin....
    y = (-y) + WIN_HEIGHT/2.0;
    return y;
}

void physical_to_viewport(double px,double py,double *vx, double *vy) {
    *vx = physical_to_viewport_x(px);
    *vy = physical_to_viewport_x(py);    
}


double viewport_to_physical_x(double px) {
    // Let's assume our play space is from -10 to + 10, centered on the origin...
    double x = px;

    // now move to origin...    
    x = x - (WIN_WIDTH / (2.0 ));

    // then unscale..
    x = x / (WIN_WIDTH / (2.0*w_physical));
    return x;
}

double viewport_to_physical_y(double py) {
    // Let's assume our play space is from -10 to + 10, centered on the origin...
    double y = py;

    // now move to origin...    
    y = y - (WIN_HEIGHT / (2.0 ));

    // then unscale..
    y = - y / (WIN_HEIGHT / (2.0*h_physical));    
    
    return y;
}

void viewport_to_physical(double vx,double vy,double *px, double *py) {
    *px = viewport_to_physical_x(vx);
    *py = viewport_to_physical_y(vy);    
}

void get_rcoords(column_vector v,int* x,int* y) {
       double vx0 = physical_to_viewport_x(v(0));
       double vy0 = physical_to_viewport_y(v(1)); 
       *x = (int) std::round(vx0);
       *y = (int) std::round(vy0);
}
//The window we'll be rendering to
SDL_Window* gWindow = NULL;
    
void draw_rect(SDL_Renderer* renderer,SDL_Rect r) {
    // Set render color to red ( background will be rendered in this color )
    SDL_SetRenderDrawColor( renderer, 255, 255, 255, 255 );

    // Clear winow
    SDL_RenderClear( renderer );

    // Set render color to blue ( rect will be rendered in this color )
    SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );

    // Render rect
    SDL_RenderFillRect( renderer, &r );

    // Render the rect to the screen
    SDL_RenderPresent(renderer);
}
void render_vector(SDL_Renderer* renderer, 
		   column_vector v0, column_vector v1) {
       double vx0 = physical_to_viewport_x(v0(0));
       double vy0 = physical_to_viewport_y(v0(1));       

       double vx1 = physical_to_viewport_x(v1(0));
       double vy1 = physical_to_viewport_y(v1(1));       

       int x0 = (int) std::round(vx0);
       int y0 = (int) std::round(vy0);
       
       int x1 = (int) std::round(vx1);
       int y1 = (int) std::round(vy1);
       
       SDL_RenderDrawLine(renderer, x0, y0, x1, y1);
}

void render_from_angle(SDL_Renderer* renderer, column_vector bk,column_vector hd, double angle) {
  column_vector bk1 = rotate_point<double>(hd,bk,angle);

    double bkx0 = bk1(0);
    double bky0 = bk1(1);
    // double bkx0_v;
    // double bky0_v;
    // physical_to_viewport(bkx0,bky0,&bkx0_v,&bky0_v);
    double bkx0_v = physical_to_viewport_x(bkx0); 
    double bky0_v = physical_to_viewport_y(bky0);
    
    int bx0 = (int) std::round(bkx0_v);
    int by0 = (int) std::round(bky0_v);
    double vx1 = physical_to_viewport_x(hd(0));
    double vy1 = physical_to_viewport_y(hd(1));       
    int x1 = (int) std::round(vx1);
    int y1 = (int) std::round(vy1);
       
    SDL_RenderDrawLine(renderer, bx0, by0, x1, y1);
}
void render_arrow(SDL_Renderer* renderer, 
		  column_vector v0, column_vector v1,double headf = 0.2) {
  double vx0 = physical_to_viewport_x(v0(0));
  double vy0 = physical_to_viewport_y(v0(1));       

  double vx1 = physical_to_viewport_x(v1(0));
  double vy1 = physical_to_viewport_y(v1(1));       
       

  int x0 = (int) std::round(vx0);
  int y0 = (int) std::round(vy0);
       
  int x1 = (int) std::round(vx1);
  int y1 = (int) std::round(vy1);
       
  SDL_RenderDrawLine(renderer, x0, y0, x1, y1);
  // Now to draw the arrow head we create the vector pointing backwards...

  column_vector bk = (v1 - v0)*headf + v1;
  
  double angle = 150*PI/180;
  render_from_angle(renderer,bk,v1,angle);
  render_from_angle(renderer,bk,v1,-angle);
  
  SDL_RenderDrawLine(renderer, x0, y0, x1, y1);      
}
void render_coords(SDL_Renderer* renderer, 
       column_vector* coords, int i, int j) {
       column_vector v0 = coords[i];
       column_vector v1 = coords[j];
       SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);       
       render_vector(renderer,v0,v1);
}



void draw_net(SDL_Renderer* renderer,  TriLadder *global_truss,column_vector* coords) {

  // need to develop complete rendering, but will do something
    // halfway at present...
     for(int j = 0; j < global_truss->num_nodes; j++) {
       int k = j - 1;
       int h = j - 2;
       if (k >= 0) {
	 	 render_coords(renderer,coords,j,k);
       }
       if (h >= 0) {
	 	 render_coords(renderer,coords,j,h);
       }
       SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
       int x;
       int y;
       get_rcoords(coords[j],&x,&y);
       //       cout << x << " " << y << "\n";
       SDL_Rect srcrect;
       srcrect.x = x-3;
       srcrect.y = y-3;
       srcrect.w = 6;
       srcrect.h = 6;
       
       SDL_RenderDrawRect(renderer, &srcrect);
       SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);       
     }

     SDL_SetRenderDrawColor(renderer, 255, 255, 0, 200);

     for(int i = 0; i < global_truss->var_edges; i++) {
       column_vector d_full(2);
       d_full = 0.0,0.0;
       int e = i + 1;
       
       for(int j = 0; j < global_truss->goals.size(); j++) {
	   column_vector d;
	   if ((e % 2) == 1) {
	     d = global_truss->compute_external_effector_derivative_c(coords,e,global_truss->goal_nodes[j]);
	   } else  if ((e % 2) == 0) {
	     d = global_truss->compute_internal_effector_derivative_c(coords,e,global_truss->goal_nodes[j]);
	   }

	   // This helps to catch internal coding errors in finding the derivatives.
	   if ((d(0) > CRISIS_DERIVATIVE_LEVEL) || (abs(d(1)) > CRISIS_DERIVATIVE_LEVEL)) {
	     cout << "CRISIS!\n";
	     print_vec(d);
	     abort();
	   }
	   d_full = d_full + (d * global_truss->goal_weights[j]);
       }


       // Note: This is experimental

      double d_obst = 0.0;
      // first we will run through all nodes.
      int first_node = global_truss->large_node(e);
      // Note this creates a n^2 operation in the number of nodes---
      // this is ripe for optimization.
      for(int j = first_node; j < global_truss->num_nodes; j++) {
	// Now, does the partial derivative of this node exist?
	double di = distance_2d(coords[j],global_truss->obstacle.center);
	double p = global_truss->obstacle.partial(di);
	if (p != 0.0) {
	  cout << "Node " << j << " derivative: " << p << "\n";
	  column_vector nd = coords[j];
	  cout << "nd :" << "\n";
	  print_vec(nd);
	  column_vector d;
	  cout << "e :" << e << "\n";
	  if ((e % 2) == 1) {
	    d = global_truss->compute_external_effector_derivative_c(coords,e,j);
	  } else  if ((e % 2) == 0) {
	    d = global_truss->compute_internal_effector_derivative_c(coords,e,j);
	  }
	  cout << " motion deivative \n";
	  print_vec(d);
	  // now d is a direction vector, we dot it with (nd - c)...
	  column_vector n_to_center = global_truss->obstacle.center - nd;
	  double direction = dot(n_to_center,d);
	  cout << "value " << j << " " << direction * p << "\n";
	  //	  d_obst += d_obst + obstacle.weight * direction * p;
	  d_obst += (direction * p);
	}
      }
       
       //       cout << "full: ";
       //       print_vec(d_full);
       int node = global_truss->large_node(e);
       column_vector tail = (coords[node]+ coords[global_truss->small_node(e)])/ 2.0;
       if ((e % 2) == 1) {
	 SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);       	 	 
       } else  if ((e % 2) == 0) {
	 SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);       	 	  
       }
       render_arrow(renderer,tail,d_full+tail);

       SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);       	 	         
       render_arrow(renderer,tail,d_obst+tail);
     }
}

void draw_physical_line(SDL_Renderer* renderer,double px0,double py0,double px1,double py1) {
      {
       double vx0 = physical_to_viewport_x(px0);
       double vy0 = physical_to_viewport_y(py0);       
       double vx1 = physical_to_viewport_x(px1);
       double vy1 = physical_to_viewport_y(py1);       
       SDL_RenderDrawLine(renderer, vx0, vy0, vx1, vy1);       
      }
}

void draw_axes(SDL_Renderer* renderer) {
      SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
      draw_physical_line(renderer,-w_physical,0.0,w_physical,0.0);
      draw_physical_line(renderer,0.0,h_physical,0.0,-h_physical);      
}
int sdl_EventFilter(void*      userdata,
                    SDL_Event* event) {
  int* mydata = (int *) userdata;
  cout << "filter:" <<  mydata[0] << "\n";
  cout << "type:" <<  event->type << "\n";
  if (event->type == SDL_MOUSEBUTTONDOWN) {
    return 1;
  } else {
    return 1;
  }
}

void mousedown_function() {
}

typedef int32_t s32;


// Thanks to: Scotty Stepens https://stackoverflow.com/questions/38334081/howto-draw-circles-arcs-and-vector-graphics-in-sdl
void DrawCircle(SDL_Renderer *Renderer, s32 _x, s32 _y, s32 radius)
{ 
   s32 x = radius - 1;
   s32 y = 0;
   s32 tx = 1;
   s32 ty = 1;
   s32 err = tx - (radius << 1); // shifting bits left by 1 effectively
                                 // doubles the value. == tx - diameter
   while (x >= y)
   {
      //  Each of the following renders an octant of the circle
      SDL_RenderDrawPoint(Renderer, _x + x, _y - y);
      SDL_RenderDrawPoint(Renderer, _x + x, _y + y);
      SDL_RenderDrawPoint(Renderer, _x - x, _y - y);
      SDL_RenderDrawPoint(Renderer, _x - x, _y + y);
      SDL_RenderDrawPoint(Renderer, _x + y, _y - x);
      SDL_RenderDrawPoint(Renderer, _x + y, _y + x);
      SDL_RenderDrawPoint(Renderer, _x - y, _y - x);
      SDL_RenderDrawPoint(Renderer, _x - y, _y + x);

      if (err <= 0)
      {
         y++;
         err += ty;
         ty += 2;
      }
      else if (err > 0)
      {
         x--;
         tx += 2;
         err += tx - (radius << 1);
      }
   }
}
// SDL doesn't have a circle drawing routine...
void render_circle(SDL_Renderer* renderer,double px, double py, double radius) {
  // now we will create a box containing the circle in physical space...
  double r = radius;
  double x0 = px - r;
  //  double y0 = py - r;
  double x1 = px + r;
  double vx0 = physical_to_viewport_x(x0);
  double vx1 = physical_to_viewport_x(x1);  
  
  double vx = physical_to_viewport_x(px);
  double vy = physical_to_viewport_y(py);
  
  //  physical_to_viewport(px,py,&vx,&vy);
  DrawCircle(renderer,vx,vy,std::abs(vx0-vx1)/2);
}



SDL_Renderer* renderer = NULL;
SDL_Window* window = NULL;
void render_all(TriLadder *an,column_vector* coordsx,Obstacle obstacle) {
  // Clear winow
  //	SDL_RenderClear( renderer );
  SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );
  SDL_RenderClear(renderer);
  draw_axes(renderer);
  // now we want to try to find coordinates in the physical space...

  draw_net(renderer,an,coordsx);
  // Here we render the obstacle
  render_circle(renderer,obstacle.center(0),obstacle.center(1),obstacle.radius);

  SDL_RenderPresent(renderer);
}

Input::Input(void(*f)())
{
  buttondown = f;
  
  mouseDown = false;
  
   m_windowClosed = false;
   

   // Make sure all the keys are set to false.
   for (int i = 0; i < 323; i++)
   {
      m_keysHeld[i] = false;
   }
}

Input::~Input()
{
}

void Input::readInput()
{
   if (SDL_PollEvent(&m_event))
   {
      if (m_event.type == SDL_QUIT)
      {
         m_windowClosed = true;
      }

      if (m_event.type == SDL_MOUSEBUTTONDOWN)
      {
	 mouseDown = true;
	 x = m_event.button.x;
	 y = m_event.button.y;
      }

      if (m_event.type == SDL_MOUSEBUTTONUP)
      {
	 mouseDown = false;	 
      }
      if (m_event.type == SDL_KEYDOWN)
      {
         m_keysHeld[m_event.key.keysym.sym] = true;
	 sdl_keycode = m_event.key.keysym.sym;
      }

      if (m_event.type == SDL_KEYUP)
      {
         m_keysHeld[m_event.key.keysym.sym] = false;
	 sdl_keycode = SDLK_a;
      }
   }
}

bool* Input::getInput()
{
   return m_keysHeld;
}

bool Input::windowClosed()
{
   return m_windowClosed;
}

void init_renderer(TriLadder *an,column_vector* coordsx,Obstacle obstacle) {


  window = SDL_CreateWindow
    (
     "Actuator Network Optimization", SDL_WINDOWPOS_UNDEFINED,
     SDL_WINDOWPOS_UNDEFINED,
     WIN_WIDTH,
     WIN_HEIGHT,
     SDL_WINDOW_SHOWN
     );
  renderer =  SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED);
  render_all(an,coordsx,obstacle);    
}

void close_renderer() {
  SDL_DestroyWindow(window);
  SDL_Quit();
}

void handle_goal_target(TriLadder *an,  column_vector* coordsx,int x, int y) {
  cout << "SPUD-C\n";
  column_vector gl(2);
  cout << "SPUD-D\n";	
  gl(0) = viewport_to_physical_x(x);
  gl(1) = viewport_to_physical_y(y);
  handle_goal_target(an,coordsx,gl);
}

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
