//Using SDL and standard IO
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include <limits.h>
#include "TriLadder.h"


#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace dlib;


void physical_to_viewport(double px,double py,double *vx, double *vy);
void viewport_to_physical(double px,double py,double *vx, double *vy);


#define LADDER_NODES 12
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5

#define USE_DERIVATIVES 1

void get_rcoords(column_vector v,int* x,int* y) {
       double px0 = v(0);
       double py0 = v(1);
       double vx0;
       double vy0; 
       physical_to_viewport(px0,py0,&vx0,&vy0);
       
       *x = (int) std::round(vx0);
       *y = (int) std::round(vy0);
}

// This is kludge -- not gettin find_min to work very well.
// WARNING: This is probably a memory leak
double* best_distances = new double[(LADDER_NODES-3)*2 +3];    

double best_score;

class Invert {
public:

  TriLadder *an;
  static TriLadder *cur_an;  
  Invert() {
  }
  
  // This function weights how close our values are to the goals.
  // At present, this is literally of the unweighted sum of the distances,
  // or th l2_norms between each goal node and its corresponding position.
  // For the purposes of the paper I will call this the "simple distance score" or sds.
  double operator() ( const column_vector& ds) const
  {
    cur_an = an;
    return objective(ds);
  }

  void set_cur_an() {
    cur_an = an;
  }

  // This is currently computing the sum of the l2_norm of the goal point quandrances (square of distance).
  static double objective(const column_vector& ds) {
    //    if (debug) std::cout << "CUR_AN " << cur_an << "\n";
    if (debug) std::cout << "OBJECTIVE INPUTS" << std::endl;
    for (int i = 0; i < cur_an->var_edges; ++i) {
           if (debug) std::cout << i << " : " << ds(i) << std::endl;
	  cur_an->distance(i+1) = ds(i);
    }

    //    if (debug) std::cout << "DISTANCES" << std::endl;
    
    column_vector coords[LADDER_NODES];

    // If I don't change an here, I'm not changing the coords!!
    find_all_coords(cur_an,coords);
    for(int i = 0; i < cur_an->num_nodes; i++) {
       std::cout << " coords["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
    }
    
    double v = 0.0;
    for(int i = 0; i < cur_an->goal_nodes.size(); i++) {
      int idx = cur_an->goal_nodes[i];
      if (debug) std::cout << "idx " <<  idx  << std::endl;      
      column_vector g = cur_an->goals[i];
      column_vector c = coords[idx];      
      column_vector x(2);
      x(0) = g(0);
      x(1) = g(1);
      column_vector y(2);
      y(0) = c(0);
      y(1) = c(1);
      
      column_vector d(2);
      if (debug) std::cout << "Invert x " <<  x(0) <<  "," << x(1)  << std::endl;
      if (debug) std::cout << "Invert y " <<  y(0) <<  "," << y(1)  << std::endl;                  
      d = x - y;
      if (debug) std::cout << "Invert d " <<  d(0) <<  "," << d(1) <<  " " << std::endl;
      v += distance_2d(x,y);
      //      v += l2_norm(d);
    }
   std::cout << "Invert v " <<  v <<  " " << std::endl;

   if (v < best_score) {
     cout << "found best: " << v << "\n";
    for (int i = 0; i < cur_an->var_edges; ++i) {
      best_distances[i] = ds(i);
    }
    best_score = v;
     
   }
    return v;    
  }

  // compute the derivatives of the objective as the configuration ds.
  // NOTE: At present this only works with ONE goal node
  static column_vector derivative(const column_vector& ds) {
    for (int i = 0; i < cur_an->var_edges; ++i) {
      if (debug) std::cout << i << " : " << ds(i) << std::endl;
      // This is correct?  It should it just be "i"?
      cur_an->distance(i+1) = ds(i);
    }
    // If I don't change an here, I'm not changing the coords!!
    column_vector coords[LADDER_NODES];    
    find_all_coords(cur_an,coords);
    column_vector d(cur_an->var_edges);

    column_vector g = cur_an->goals[0];
    int idx = cur_an->goal_nodes[0];	
    column_vector c = coords[idx];
      
    for(int i = 0; i < cur_an->var_edges; i++) {
      // The true edge number is one higher than the index of the variable edges, since the first is fixed.
      int e = i + 1;
      column_vector dx(2);
      if (((e % 2) == 1)) {
  	dx = cur_an->compute_external_effector_derivative_c(coords,e);
      }
      else if ((e % 2) == 0) {
      	  dx = cur_an->compute_internal_effector_derivative_c(coords,e);
      }
      // Print out these values until we understand what is hapening here.
      cout << " c, g " << c << " , " << g << "\n";
      column_vector goal_direction = c - g;
      cout << "dx, direction: " << "\n";
      print_vec(dx);      
      print_vec(goal_direction);
      double prod = dot(goal_direction,dx);
      cout << " product : " << prod << "\n";
      d(i) = prod;
    }
    cout << "cur_an->var_edges = " << cur_an->var_edges << "\n";

    cout << "DERIVATIVES" << "\n";
    for(int i = 0; i < cur_an->var_edges; i++) {
      cout << "i " << i << " " << d(i) << "\n";
    }

    return d;
  }
  
};

TriLadder *Invert::cur_an = 0;


void solve_inverse_problem(TriLadder *an) {
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
  std::cout << "OUT LOOP" <<  std::endl;
  
  Invert inv;
  inv.an = an;
  inv.set_cur_an();

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
    std::cout << "inv(x) " << inv(sp) << std::endl;
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
			      bfgs_search_strategy(),
			     // lbfgs_search_strategy(30),
			     // cg_search_strategy(),
			     //			     newton_search_strategy,
			     objective_delta_stop_strategy(1e-5),
			     *Invert::objective,
			     *Invert::derivative,
			     sp,
			     uniform_matrix<double>(n,1, LOWER_BOUND),  // lower bound constraint
			     uniform_matrix<double>(n,1, UPPER_BOUND)   // upper bound constraint
			     );
    cout << "===============\n";
    cout << "score = " << score << "\n";
    cout << "best  = " << best_score << "\n";
    for (int i = 0; i < an->var_edges; ++i) {
      cout << i << " " << best_distances[i] << "\n";
      an->distance(i+1) = best_distances[i];

      cout << i << " " << sp(i) << "\n";
    }
    // WARNING: we should free best here...
  }

};


// ----------------------------------------------------------------------------------------

int mainx(TriLadder *an,column_vector* coords)
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
	  solve_inverse_problem(an);

	  for (int i = 0; i < an->num_edges; ++i) {
	    std::cout << i << " : " << an->distance(i) << std::endl;
	  }

	  find_all_coords(an,coords);
	  Invert inv;
	  inv.an = an;
	  inv.set_cur_an();
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
       double px0 = v0(0);
       double py0 = v0(1);
       double vx0;
       double vy0; 
       physical_to_viewport(px0,py0,&vx0,&vy0);
       
       double px1 = v1(0);
       double py1 = v1(1);
       double vx1;
       double vy1; 
       physical_to_viewport(px1,py1,&vx1,&vy1);

       int x0 = (int) std::round(vx0);
       int y0 = (int) std::round(vy0);
       
       int x1 = (int) std::round(vx1);
       int y1 = (int) std::round(vy1);
       
       SDL_RenderDrawLine(renderer, x0, y0, x1, y1);
}
void render_arrow(SDL_Renderer* renderer, 
		  column_vector v0, column_vector v1,double headf = 0.2) {
       double px0 = v0(0);
       double py0 = v0(1);
       double vx0;
       double vy0; 
       physical_to_viewport(px0,py0,&vx0,&vy0);
       
       double px1 = v1(0);
       double py1 = v1(1);
       double vx1;
       double vy1; 
       physical_to_viewport(px1,py1,&vx1,&vy1);

       int x0 = (int) std::round(vx0);
       int y0 = (int) std::round(vy0);
       
       int x1 = (int) std::round(vx1);
       int y1 = (int) std::round(vy1);
       
       SDL_RenderDrawLine(renderer, x0, y0, x1, y1);
       // Now to draw the arrow head we create the vector pointing backwards...
       {
       column_vector bk = (v1 - v0)*headf + v1;
       column_vector bk1 = rotate_point<double>(v1,bk,150*PI/180);

       double bkx0 = bk1(0);
       double bky0 = bk1(1);
       double bkx0_v;
       double bky0_v;
       physical_to_viewport(bkx0,bky0,&bkx0_v,&bky0_v);       
       int bx0 = (int) std::round(bkx0_v);
       int by0 = (int) std::round(bky0_v);
       
       SDL_RenderDrawLine(renderer, bx0, by0, x1, y1);
       }
       {
       column_vector bk = (v1 - v0)*headf + v1;
       column_vector bk1 = rotate_point<double>(v1,bk,-150*PI/180);

       double bkx0 = bk1(0);
       double bky0 = bk1(1);
       double bkx0_v;
       double bky0_v;
       physical_to_viewport(bkx0,bky0,&bkx0_v,&bky0_v);       
       int bx0 = (int) std::round(bkx0_v);
       int by0 = (int) std::round(bky0_v);
       
       SDL_RenderDrawLine(renderer, bx0, by0, x1, y1);
       }
       // Then we shorten it to the head lenghth..
       // then we rotate and render 45 degrees in each direction.
       SDL_RenderDrawLine(renderer, x0, y0, x1, y1);       
}
void render_coords(SDL_Renderer* renderer, 
		   column_vector* coords, int i, int j) {
       column_vector v0 = coords[i];
       column_vector v1 = coords[j];
       SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);       
       render_vector(renderer,v0,v1);
}

const int WIN_WIDTH = 640 * 2;
const int WIN_HEIGHT = 480 * 2;

// These are the size of the physical size
//const double w = (LADDER_NODES < 11) ? 10.0 : (LADDER_NODES < 21) ? 20.0 : 40.0;
//const double h = (LADDER_NODES < 11) ? 10.0 : (LADDER_NODES < 21) ? 20.0 : 40.0;

const double w = (LADDER_NODES / 10.0 ) * 12.0 ;
const double h = (LADDER_NODES / 10.0 ) * 12.0 ;


void draw_net(SDL_Renderer* renderer,  TriLadder *an,column_vector* coords) {
    // Set render color to red ( background will be rendered in this color )
    // SDL_SetRenderDrawColor( renderer, 255, 255, 255, SDL_ALPHA_OPAQUE );

    // Clear winow
    //    SDL_RenderClear( renderer );

    // Set render color to blue ( rect will be rendered in this color )
    //    SDL_SetRenderDrawColor( renderer, 0, 0, 0, SDL_ALPHA_OPAQUE );
    //    SDL_RenderClear(renderer);

    // need to develop complete rendering, but will do something
    // halfway at present...
     for(int j = 0; j < an->num_nodes; j++) {
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
       cout << x << " " << y << "\n";
       SDL_Rect srcrect;
       srcrect.x = x-3;
       srcrect.y = y-3;
       srcrect.w = 6;
       srcrect.h = 6;
       
       SDL_RenderDrawRect(renderer, &srcrect);
       SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);       
     }

     SDL_SetRenderDrawColor(renderer, 255, 255, 0, 200);       
     for(int i = 0; i < an->var_edges; i++) {
       //       column_vector d = an->compute_single_derivative_c(coords,i);
       int e = i + 1;
       column_vector d;
       if ((e % 2) == 1) {
	 d = an->compute_external_effector_derivative_c(coords,e);
	 SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);       	 	 
       } else  if ((e % 2) == 0) {
	  d = an->compute_internal_effector_derivative_c(coords,e);
	  SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);       	 	  
       }
       print_vec(d);
       int node = an->large_node(e);
       column_vector tail = (coords[node]+ coords[an->small_node(e)])/ 2.0;
       render_arrow(renderer,tail,d+tail);
     } 
}

void draw_axes(SDL_Renderer* renderer) {
      SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
      {
       double px0 = -w;
       double py0 = 0.0;
       double vx0;
       double vy0; 
       physical_to_viewport(px0,py0,&vx0,&vy0);
       
       double px1 = w;
       double py1 = 0.0;
       double vx1;
       double vy1; 
       physical_to_viewport(px1,py1,&vx1,&vy1);
       SDL_RenderDrawLine(renderer, vx0, vy0, vx1, vy1);       
      }
      {
       double px0 = 0.0;
       double py0 = h;
       double vx0;
       double vy0; 
       physical_to_viewport(px0,py0,&vx0,&vy0);
       
       double px1 = 0.0;
       double py1 = -h;
       double vx1;
       double vy1; 
       physical_to_viewport(px1,py1,&vx1,&vy1);
       SDL_RenderDrawLine(renderer, vx0, vy0, vx1, vy1);       
      }

}


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
	//         m_keysHeld[m_event.key.keysym.sym] = true;
	 mouseDown = true;
	 x = m_event.button.x;
	 y = m_event.button.y;

	 cout << "XXXXXXXXXXXXXXXXXXXXX\n";
	 
	 //	 cout << x << "," << y << "\n";

      }

      if (m_event.type == SDL_MOUSEBUTTONUP)
      {
	//         m_keysHeld[m_event.key.keysym.sym] = false;
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


void physical_to_viewport(double px,double py,double *vx, double *vy) {
  
    // Let's assume our play space is from -10 to + 10, centered on the origin...
 
    double x = px;
    double y = py;
    // first scale appropriately
    x = x * (WIN_WIDTH / (2.0 * w));
    y = y * (WIN_HEIGHT / (2.0 * h));
    
    // now move to origin....
    x += WIN_WIDTH/2.0;
    y = (-y) + WIN_HEIGHT/2.0;

    // These adjust our weird grid background to the origin...
    //    y = y + WIN_HEIGHT / (2.0 *(2.0 * h));
    //    x = x + WIN_WIDTH / (2.0 * (2.0 * w)) ;
    
    *vx = x;
    *vy = y;
}

void viewport_to_physical(double px,double py,double *vx, double *vy) {
  
    // Let's assume our play space is from -10 to + 10, centered on the origin...
    double x = px;
    double y = py;

        // now move to origin...    
    x = x - (WIN_WIDTH / (2.0 ));
    y = y - (WIN_HEIGHT / (2.0 ));

    // then unscale..
    x = x / (WIN_WIDTH / (2.0*w));
    y = - y / (WIN_HEIGHT / (2.0*h));    
    
    *vx = x;
    *vy = y;
}

void render_all(SDL_Renderer* renderer, TriLadder *an,column_vector* coordsx) {
  	// Clear winow
  	//	SDL_RenderClear( renderer );
  SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );
  SDL_RenderClear(renderer);
  draw_axes(renderer);
  // now we want to try to find coordinates in the physical space...
	

  draw_net(renderer,an,coordsx);
  SDL_RenderPresent(renderer);       				
}

int main( int argc, char* args[] )
{
  {
    	double vx = 0.0;
  	double vy = 0.0;
  	double px;
  	double py;
  	viewport_to_physical(vx,vy,&px,&py);
  	cout << "PPPPPPPPPPPPPPPPP\n";
  	cout << vx << "," << vy << "\n";	
  	cout << px << "," << py << "\n";
  	physical_to_viewport(px,py,&vx,&vy);	
  	cout << vx << "," << vy << "\n";	
  }
  TriLadder an = TriLadder(LADDER_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
			   );

  column_vector* coordsx = new column_vector[an.num_nodes];

  mainx(&an,coordsx);

  // for(int i = 0; i < an.num_nodes; i++) {
  //   cout << "X" <<  i << "\n";
  //   print_vec(coordsx[i]);
  // }
  // cout << '\n';
  
    SDL_Window* window = NULL;
    window = SDL_CreateWindow
    (
        "Actuator Network Optimization", SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        WIN_WIDTH,
        WIN_HEIGHT,
        SDL_WINDOW_SHOWN
    );

    // Setup renderer
    SDL_Renderer* renderer = NULL;
    renderer =  SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED);

    render_all(renderer,&an,coordsx);
	
    SDL_Event spud;

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


  	double vx = input.x;
  	double vy = input.y;
  	double px;
  	double py;
  	viewport_to_physical(vx,vy,&px,&py);
  	column_vector gl(2);
  	gl(0) = px;
  	gl(1) = py;
  	an.goals[0] = gl;
	cout  << "goal : " << gl << "\n";
	
  	mainx(&an,coordsx);

	render_all(renderer,&an,coordsx);
       
  	input.mouseDown = false;
      }
      SDL_Delay( 10 );
      n++;
    }
    SDL_WaitEvent(&spud);
    cout << "type" << spud.type << "\n";

    SDL_DestroyWindow(window);
    SDL_Quit();
    
    return EXIT_SUCCESS;
}
