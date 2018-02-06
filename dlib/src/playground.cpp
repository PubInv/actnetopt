// playground.cpp -- Interactive tool for testing configuration of a robotic manipulator via gradient techniquies 
// Copyright (C) Robert L. Read, 2018
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#include "playground.h"
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include <limits.h>
#include "Invert.h"
#include "Obstacle.h"
#include "TriLadder.h"
#include <math.h>

// ToDos:
// *) Clean up the code
// *) Implement a circle render for drawing the obstacle
// *) Support multiple obstacles
// *) Create an API-level separation between the playground/rendered and the solver!

using namespace std;
using namespace dlib;

#define TRUSS_NODES 30
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5
#define CRISIS_DERIVATIVE_LEVEL 1000.0

#define USE_DERIVATIVES 1

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


// Sadly, the way the objective and deriviate functions are called
// the do not allow general "client" data to be passed in, and they are static.
// So keeping a global variable is the only effective way to maintain information
// for solving the "forward" problem.
// Therefore "global_truss" is global.
TriLadder *Invert::global_truss = 0;

Obstacle obstacle;


void solve_inverse_problem(TriLadder *an,Obstacle ob) {
  column_vector sp(an->var_edges);
  column_vector lb(an->var_edges);
  column_vector ub(an->var_edges);      

  // We need the TriLadder to have distances in order to iniitilize this meaningfully
  for (int i = 0; i < an->var_edges; i++) {
    // This assumes the first edge is fixed, which it is in TriLadder
    sp(i) = an->distance(i + 1);
    lb(i) = an->lower_bound(i);
    ub(i) = an->upper_bound(i);    
  }
  
  Invert inv;
  inv.an = an;
  inv.set_global_truss(obstacle);
  inv.ob = ob;

  int n = an->var_edges;

  if (!USE_DERIVATIVES) {
    // Inv is the objective funciton/object. It computes didstance for goal node.
    find_min_bobyqa(inv, 
		    sp, 
		    (n+1)*(n+2)/2,    // number of interpolation points
		    uniform_matrix<double>(n,1, LOWER_BOUND),  // lower bound constraint
		    uniform_matrix<double>(n,1, UPPER_BOUND),   // upper bound constraint
		    INITIAL/5,    // initial trust region radius (rho_begin)
		    1e-5,  // stopping trust region radius (rho_end)
		    10000   // max number of objective function evaluations
		    );
    for (int i = 0; i < an->var_edges; i++) {
      if (debug)  std::cout << "distance edge" << i+1 << " :  " << sp(i) << std::endl;
      an->distance(i+1) = sp(i);
   }

   if (debug)  std::cout << "inv(x) " << inv(sp) << std::endl;      
  } else {
    best_score = std::numeric_limits<float>::max();

    for (int i = 0; i < an->var_edges; ++i) {
	   best_distances[i] = an->distance(i+1);
    }
    
    double score = find_min_box_constrained(
      			    // bfgs_search_strategy(),
			     lbfgs_search_strategy(30),
			     // cg_search_strategy(),
			     //			     newton_search_strategy,
			     objective_delta_stop_strategy(1e-5),
			     *Invert::objective,
			     *Invert::derivative,
			     sp,
			     uniform_matrix<double>(n,1, LOWER_BOUND),  // lower bound constraint
			     uniform_matrix<double>(n,1, UPPER_BOUND)   // upper bound constraint
			     );

    // I uses this to get rid of the warning
    score = score + 0.0;
    for (int i = 0; i < an->var_edges; ++i) {
      an->distance(i+1) = best_distances[i];
    }
  }

};


// ----------------------------------------------------------------------------------------

int mainx(TriLadder *an,column_vector* coords,Obstacle ob)
{
    try
    {
      // NOTE: This is currently using an algorithm that works without derivatives,
      // even though I think I have correctly analyzed the derivatives (see LaTeX paper.)
      // It can probably be made more efficient to use a dlib algorithm that uses derivatives.
      // However, that is a low priority until I get the playground working.
	{

	  for (int i = 0; i < an->num_edges; ++i) {
	    an->distance(i) = MEDIAN;
	  }

	  // WARNING: We are trying here to skip over fixed edge; this should really be computed!
	  for (int i = 1; i < an->num_edges; ++i) {
	    an->distance(i) = INITIAL;
	  }

	  //	  column_vector* coordsx = new column_vector[an->num_nodes];
	  
	  find_all_coords(an,coords);
	  solve_inverse_problem(an,ob);

	  for (int i = 0; i < an->num_edges; ++i) {
	    //	    std::cout << "mainx" << i << " : " << an->distance(i) << std::endl;
	  }

	  find_all_coords(an,coords);
	  Invert inv;
	  inv.an = an;
	  inv.set_global_truss(obstacle);
	  column_vector sp(an->var_edges);
	  for (int i = 0; i < an->var_edges; i++) {
	    sp(i) = an->distance(i + 1);
	  }	  
	  // double final = inv(sp);
	  // std::cout << "inv(x) final : " << final << std::endl;
	  // for(int i = 0; i < an->num_nodes; i++) {
	  //   std::cout << " d["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
	  // }
	  // should now deallocate coords
	}
    }
    catch (std::exception& e)
    {
        cout << e.what() << endl;
    }
    return 0;
}

//The window we'll be rendering to
SDL_Window* gWindow = NULL;
    
//The surface contained by the window
SDL_Surface* gScreenSurface = NULL;

//The image we will load and show on the screen
SDL_Surface* gHelloWorld = NULL;

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


void render_all(SDL_Renderer* renderer, TriLadder *an,column_vector* coordsx,Obstacle obstacle) {
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


// TODO: reorganized this so that initialization is in a separate routine

TriLadder *init_TriLadder() {

  // This needs to be generalized but for now I'm going to
  // just add an obstacle.  If I can successfully work around
  // an obstacle, that will be strong evidence that I have a valuable system.
  obstacle.radius = 2.0;
  obstacle.center = column_vector(2);
  obstacle.center = -2.0, 10.0;
  obstacle.weight = 4.0;

  TriLadder *an = new TriLadder(TRUSS_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
			   );

  int last_node = an->num_nodes - 1;
  double bx = ((last_node % 2) == 0) ? 0.0 : an->median_d * cos(30.0*M_PI/180);
  double by = (an->median_d/2.0) * last_node;

  // These are examples of adding different goals...
  //  an->add_goal_node(an->num_nodes*1/3,1*bx/3+-2.0,1*by/3,0.5);
  //  an->add_goal_node(an->num_nodes*2/3,2*bx/3+ 5.0,2*by/3,0.5);  
  double mx = 1.0;
  double my = 1.0;
  an->add_goal_node(an->num_nodes-1,bx+mx,by+my,1.0);
  
  if (debug) {
    for(int i = 0; i < an->goal_nodes.size(); i++) {
      cout << "goal_nodes[" << i << "] " << an->goal_nodes[i] << "\n";
    }
  }
  return an;
}

SDL_Renderer* renderer = NULL;
SDL_Window* window = NULL;

void handle_goal_target(TriLadder *an,  column_vector* coordsx,int x, int y) {
  cout << "SPUD-C\n";
    	column_vector gl(2);
  cout << "SPUD-D\n";	
  	gl(0) = viewport_to_physical_x(x);
  	gl(1) = viewport_to_physical_y(y);
  cout << "SPUD-E\n";		

  	an->goals[an->goals.size() - 1] = gl;
  cout << "SPUD-F\n";			
	cout  << "goal : " << gl << "\n";
  cout << "SPUD-G\n";				
	best_score = std::numeric_limits<float>::max();	
	auto start = std::chrono::high_resolution_clock::now();
	
  	mainx(an,coordsx,obstacle);
	
	auto elapsed = std::chrono::high_resolution_clock::now() - start;
	long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
	long long milliseconds = microseconds/ 1000.0;

	cout << "optimization tim: ms = " << milliseconds << "\n";
	render_all(renderer,an,coordsx,obstacle);
	//  	input.mouseDown = false;
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
    render_all(renderer,an,coordsx,obstacle);    
}

void close_renderer() {
  SDL_DestroyWindow(window);
  SDL_Quit();
    
}
