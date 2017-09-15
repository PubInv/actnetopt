//Using SDL and standard IO
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>


#include <math.h>

#define PI 3.14159265

using namespace std;
using namespace dlib;


// ----------------------------------------------------------------------------------------

// In dlib, the general purpose solvers optimize functions that take a column
// vector as input and return a double.  So here we make a typedef for a
// variable length column vector of doubles.  This is the type we will use to
// represent the input to our objective functions which we will be minimizing.
typedef matrix<double,0,1> column_vector;


// TODO: This is very messy. My goal here is to test this out.
// A) I need to use some sort of Vector.  This system uses column_vector;
// I hardely know how to use that.
// B) Maybe I should first test multiple variables in a simpler system first!
// Can I solve 100 variables?
void print_vec(column_vector& vec)
{
  for(int i = 0; i < vec.size(); i++) {
    std::cout << ' ' << vec(i);
    }
    std::cout << '\n';
}


void physical_to_viewport(double px,double py,double *vx, double *vy);
void viewport_to_physical(double px,double py,double *vx, double *vy);

const bool debug_find = false;
const bool debug = false;

double distance(column_vector a, column_vector b) {
  double d = 0.0;
  for(int i = 0; i < a.size(); i++) {
    d += (a(i)-b(i))*(a(i)-b(i));
  }
  return std::sqrt(d);
}

double l2_norm(column_vector a) {
  double d = 0.0;
  for(int i = 0; i < a.size(); i++) {
    d += a(i)*a(i);
  }
  return d;
}

#define LADDER_NODES 10
// #define VAR_EDGES (LADDER_NODES-3)*2+2;
#define VAR_EDGES 16
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5

class TriLadder {
public:

  // I know this is poor C++, I am not a very good C++ coder
  //  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
  // Make convenient labels for the vertices
  enum { A, B, C, D, E, F };
  // Making this a const seems to destry to the implicit
  // copy assignment; I have no idea why
  static const int num_nodes = LADDER_NODES;
  static const int num_edges = (num_nodes-3)*2 + 3;
  static const int var_edges = num_edges-1;
  
  const char* name = "ABCDE";

  // writing out the edges in the graph
  //  typedef std::pair<int, int> Edge;

  //  Graph g;
  //  Edge e;

  //  static const int num_edges = 3;  
  //  Edge edge_array[num_edges];
  column_vector fixed_nodes;
  int node_fixing_order[num_nodes];
  //  typedef property_map<Graph, vertex_index_t>::type IndexMap;
  //  typedef graph_traits<Graph>::vertex_iterator vertex_iter;
  //  IndexMap index;
  column_vector coords[num_nodes];
  column_vector goals[1];

  column_vector distance;
  
  column_vector lower_bound;
  column_vector upper_bound;
  
  //  Eigen::VectorXd lower_bound;
  //  Eigen::VectorXd upper_bound;  

  // This is a map into the goal position for each goal.
  // goal_nodes[a] = b => goals[a] should be considered node b.
  std::vector<int> goal_nodes;

  TriLadder() {

    // edge_array[0] = Edge(A,B);
    // edge_array[1] = Edge(B,C);
    // edge_array[2] = Edge(A,C);      
    // edge_array[3] = Edge(D,C);
    // edge_array[4] = Edge(C,E);
    // edge_array[5] = Edge(B,D);
    // edge_array[6] = Edge(D,E);
    cout << "TTT\n";
    fixed_nodes.set_size(2);
    fixed_nodes(0) = A;
    fixed_nodes(1) = B;
    cout << "TTTT1\n";	  
    //    fixed_nodes[1] = B;

    // node_fixing_order[0] = A;
    // node_fixing_order[1] = B;
    // node_fixing_order[2] = C;
    // node_fixing_order[3] = D;
    // node_fixing_order[4] = E;
    // node_fixing_order[5] = F;
    
    //    num_edges = sizeof(edge_array)/sizeof(edge_array[0]);
    
    // declare a graph object
    //    Graph gg(num_nodes);
    //    g = gg;

    // add the edges to the graph object
    distance.set_size(num_edges);
    lower_bound.set_size(num_edges);
    upper_bound.set_size(num_edges);    
    for (int i = 0; i < num_edges; ++i) {
      //      add_edge(edge_array[i].first, edge_array[i].second, g);
      //      lower_bound[i] = 1.2;
      //      upper_bound[i] = 2.0;
      distance(i) = MEDIAN;
    }
    
    for (int i = 0; i < var_edges; ++i) {
      lower_bound(i) = LOWER_BOUND;
      upper_bound(i) = UPPER_BOUND;            
    }
    int last_node = num_nodes - 1;
    double bx = ((last_node % 2) == 0) ? 0.0 : MEDIAN * cos(30.0*PI/180);
    double by = (MEDIAN/2.0) * last_node;
    // modify the relaxed position by this amount...
    double mx = 0.1;
    double my = 1.0;
    column_vector gl(2);
    gl(0) = bx+mx;
    gl(1) = by+my;
    goals[0] = gl;
    cout << "last_node = " << last_node << "\n";
    print_vec(gl);

    goal_nodes.push_back(num_nodes-1);
    
    //    index = get(vertex_index, g);
  }
  double gscore() {
    double v = 0.0;
    for(int i = 0; i < goal_nodes.size(); i++) {
      int idx = goal_nodes[i];
      if (debug) std::cout << "idx " <<  idx  << std::endl;      
      column_vector x;
      x(0) = goals[i](0);
      x(1) = goals[i](1);
      column_vector y;
      y(0) = coords[idx](0);
      y(1) = coords[idx](1);
      column_vector d;
      //      std::cout << "Invert x " <<  x(0) <<  "," << x(1)  << std::endl;
      //      std::cout << "Invert y " <<  y(0) <<  "," << y(1)  << std::endl;                  
      d = x - y;
      if (debug) std::cout << "Invert d " <<  d(0) <<  "," << d(1) <<  " " << std::endl;
      double vp = d(0)*d(0) + d(1)*d(1);
      v += vp;
    }
    return v;
  }
  double lscore() {
    return 0.0;
  }
};

typedef enum { CW, CCW } Chirality;

class FindCoords {
public:
  column_vector a;
  column_vector b;    

  TriLadder *an;
  
  // Should the third point cc or ccw from a to b?
  // In other words, we use this to disambiguate the two distance based solutions.
  Chirality chi;

  //     const double dab = 1.5; // This is in fact a constant in our frame
  double dac; // these are in fact inputs to the problem
  double dbc;
  
  // this is just the objective (NOT optional)
  // This input is an x,y position for c
  // The lengths between a, b, and c are constants (effectively, input to the problem)
  double operator() ( const column_vector& x) const
  {
    column_vector y(2);
    y  = x(0), x(1);

    column_vector bc(2);
    bc = y - b;
    
    column_vector ac(2);
    ac = y - a;

    double an = ac(0)*ac(0)+ac(1)*ac(1);
    double bn = bc(0)*bc(0)+bc(1)*bc(1);    
    return
      pow(sqrt(an) - dac,2) +
      pow(sqrt(bn) - dbc,2);
  }
};

// This is really a determinant of a 3x3 matrix with a column on the right
double sense(double a, double b, double d, double e, double g, double h) {
  return a*e - a*h - b*d + b*g + d*h - e*g;
}


// This is an attempt to use FindCoords in the correct order to find all of the things

// This this so that it doesn't modify *an but rather returns the coordinate vector

void find_all_coords(TriLadder *an,column_vector coords[]) {
    FindCoords f;

    // This should really be moved inot the TriLadder class in some way!
    column_vector temp0(2);
    temp0 = 0, 0;
    coords[0] = temp0;

    column_vector temp1(2);
    temp1 = 1.3, 0.75;
    coords[1] = temp1;
    
    // Basic structure: Iteratively find coordinates based on simple triangulations.
    // This only works for actuator networks in which we can
    int fs = an->fixed_nodes.size();
    for(int i = fs; i < an->num_nodes; i++) {
      // Let's set up initial values
      // choose a starting point

      column_vector  x(2);
      x = 1, 2;
      f.a.set_size(2);
      f.a(0) = coords[i-fs](0);
      f.a(1) = coords[i-fs](1);
      f.b.set_size(2);
      f.b(0) = coords[i-(fs-1)](0);
      f.b(1) = coords[i-(fs-1)](1);

      Chirality desired_sense = ((i % 2) == 0) ? CCW : CW;
      f.chi = desired_sense;

      // and minimize the function
      if (debug_find) std::cout << "f.a " << f.a << std::endl;
      if (debug_find) std::cout << "f.b " << f.b << std::endl;


      f.dac = an->distance(i-fs);
      f.dbc = an->distance(i-(fs-1));
      
      find_min_using_approximate_derivatives(bfgs_search_strategy(),
					     objective_delta_stop_strategy(1e-7),
					     f, x, -1);
      
      double det = sense(f.a(0), f.a(1), f.b(0), f.b(1), x(0), x(1));
      // d > 0 => ccw, d < 0 => cw.
      // now how do we make this penalize for going the wrong way?
      bool found_sense = (det > 0) ? CCW : CW;

      if (debug_find) std::cout << "processing node: " << i << std::endl;
      
      if (debug_find) std::cout << "det " << det << std::endl;
      
      bool need_to_flip = (desired_sense != found_sense);

      if (need_to_flip) {
	if (debug_find) std::cout << "NEED TO FLIP" << std::endl;
	if (debug_find) std::cout << "start: " << x << std::endl;
	column_vector r(2);
      
	// first we obtain the y = mx + b form..
	if ( (f.a(0)-f.b(0)) == 0) {
	  if (debug_find) std::cout << " SLOPE ZERO "  << std::endl;	  
	  // In this case, we have a vertical line, we can tell based on
	  // just where it is based on the x value of x and whether a is above
	  r = 2*f.a(0) - x(0),x(1);
	} else {
	  const double m = (f.a(1)-f.b(1)) / (f.a(0)-f.b(0));
	  const double b = f.a(1) - m * f.a(0);
	  const double d = (x(0) + (x(1) - b)*m)/(1+m*m);
	  const double xp = 2*d - x(0);
	  const double yp = 2*d*m - x(1) + 2 * b;
	  r = xp, yp;
	}
	if (debug_find) std::cout << "flipped: " <<  r  << std::endl;
	x(0) = r(0);
	x(1) = r(1);
      }
      if (debug_find) std::cout << " x " << x << std::endl;

      coords[i].set_size(2);
      coords[i](0) = x(0);
      coords[i](1) = x(1);
      
      // print argmin
      // Now take x and make it the first coord...
      
      if (debug_find) std::cout << "f(x) " << f(x) << std::endl;
    }
    for (int i = 0; i < an->num_nodes; i++) {
      if (debug_find) std::cout << i << " =  " << an->coords[i] << std::endl;      
    }
};



class Invert {
public:

  TriLadder *an;
  Invert() {
  }
  
  // This function weights our close our values are to the goals.  
  double operator() ( const column_vector& ds) const
  {

    if (debug) std::cout << "INPUTS" << std::endl;
    for (int i = 0; i < an->var_edges; ++i) {
          if (debug) std::cout << i << " : " << ds(i) << std::endl;
	  an->distance(i+1) = ds(i);
    }

    if (debug) std::cout << "DISTANCES" << std::endl;
    
    column_vector coords[LADDER_NODES];

    // If I don't change an here, I'm not changing the coords!!
    find_all_coords(an,coords);
    for(int i = 0; i < an->num_nodes; i++) {
      //      std::cout << " d["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
    }

    
    double v = 0.0;
    for(int i = 0; i < an->goal_nodes.size(); i++) {
      int idx = an->goal_nodes[i];
      if (debug) std::cout << "idx " <<  idx  << std::endl;      
      column_vector g = an->goals[i];
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
      v += l2_norm(d);
    }
    if (debug) std::cout << "Invert v " <<  v <<  " " << std::endl;
    return v;
  }
};


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

  int n = an->var_edges;
  
  find_min_bobyqa(inv, 
		  sp, 
		  (n+1)*(n+2)/2,    // number of interpolation points
		  uniform_matrix<double>(n,1, LOWER_BOUND),  // lower bound constraint
		  uniform_matrix<double>(n,1, UPPER_BOUND),   // upper bound constraint
		  INITIAL/5,    // initial trust region radius (rho_begin)
		  1e-6,  // stopping trust region radius (rho_end)
		  100000    // max number of objective function evaluations
		  );
  cout << "bobyqa solution:\n" << sp << endl;
  std::cout << "inv(x) " << inv(sp) << std::endl;    

   for (int i = 0; i < an->var_edges; i++) {
    std::cout << "distance edge" << i+1 << " :  " << sp(i) << std::endl;
    an->distance(i+1) = sp(i);
  }
  std::cout << "inv(x) " << inv(sp) << std::endl;  
  
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
	  column_vector sp(an->var_edges);
	  for (int i = 0; i < an->var_edges; i++) {
	    sp(i) = an->distance(i + 1);
	  }	  
	  double final = inv(sp);
	  std::cout << "inv(x) final : " << final << std::endl;
	  for(int i = 0; i < an->num_nodes; i++) {
	    std::cout << " d["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
	  }
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

void render_coords(SDL_Renderer* renderer, 
		   column_vector* coords, int i, int j) {
       column_vector v0 = coords[i];
       column_vector v1 = coords[j];
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

       cout << "vx,vy" << vx0 << "," << vy0 << "\n";
       cout << "vx,vy" << vx1 << "," << vy1 << "\n";
       
       int x0 = (int) std::round(vx0);
       int y0 = (int) std::round(vy0);
       
       int x1 = (int) std::round(vx1);
       int y1 = (int) std::round(vy1);
       
       SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
       SDL_RenderDrawLine(renderer, x0, y0, x1, y1);
}

const int WIN_WIDTH = 640 * 2;
const int WIN_HEIGHT = 480 * 2;

// These are the size of the physical size
const double w = 20.0;
const double h = 20.0;


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
       if (k > 0) {
	 render_coords(renderer,coords,j,k);
       }
       if (h > 0) {
	 render_coords(renderer,coords,j,h);
       }
     }
     std::cout << '\n';
     //     SDL_RenderPresent(renderer);
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
       cout << vx0 << "," <<  vx1 << "," << vy0 << "," << vy1 << "\n";
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
       cout << vx0 << "," <<  vx1 << "," << vy0 << "," << vy1 << "\n";
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
	 
	 cout << x << "," << y << "\n";

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
  TriLadder an = TriLadder();

  column_vector* coordsx = new column_vector[an.num_nodes];

  mainx(&an,coordsx);

  for(int i = 0; i < an.num_nodes; i++) {
    cout << "X" <<  i << "\n";
    print_vec(coordsx[i]);
  }
  cout << '\n';
  
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
	
	mainx(&an,coordsx);
       
	// Clear winow
	//	SDL_RenderClear( renderer );
	SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );
	SDL_RenderClear(renderer);
	draw_axes(renderer);
	// now we want to try to find coordinates in the physical space...
	

	draw_net(renderer,&an,coordsx);
	SDL_RenderPresent(renderer);       				
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
