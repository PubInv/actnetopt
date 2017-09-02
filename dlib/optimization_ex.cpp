// The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
/*

    This is an example illustrating the use the general purpose non-linear 
    optimization routines from the dlib C++ Library.

    The library provides implementations of the conjugate gradient,  BFGS,
    L-BFGS, and BOBYQA optimization algorithms.  These algorithms allow you to
    find the minimum of a function of many input variables.  This example walks
    though a few of the ways you might put these routines to use.

*/


#include <dlib/optimization.h>
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

// ----------------------------------------------------------------------------------------
// Below we create a few functions.  When you get down into main() you will see that
// we can use the optimization algorithms to find the minimums of these functions.
// ----------------------------------------------------------------------------------------

double rosen (const column_vector& m)
/*
    This function computes what is known as Rosenbrock's function.  It is 
    a function of two input variables and has a global minimum at (1,1).
    So when we use this function to test out the optimization algorithms
    we will see that the minimum found is indeed at the point (1,1). 
*/
{
    const double x = m(0); 
    const double y = m(1);

    // compute Rosenbrock's function and return the result
    return 100.0*pow(y - x*x,2) + pow(1 - x,2);
}

// This is a helper function used while optimizing the rosen() function.  
const column_vector rosen_derivative (const column_vector& m)
/*!
    ensures
        - returns the gradient vector for the rosen function
!*/
{
    const double x = m(0);
    const double y = m(1);

    // make us a column vector of length 2
    column_vector res(2);

    // now compute the gradient vector
    res(0) = -400*x*(y-x*x) - 2*(1-x); // derivative of rosen() with respect to x
    res(1) = 200*(y-x*x);              // derivative of rosen() with respect to y
    return res;
}

// This function computes the Hessian matrix for the rosen() fuction.  This is
// the matrix of second derivatives.
matrix<double> rosen_hessian (const column_vector& m)
{
    const double x = m(0);
    const double y = m(1);

    matrix<double> res(2,2);

    // now compute the second derivatives 
    res(0,0) = 1200*x*x - 400*y + 2; // second derivative with respect to x
    res(1,0) = res(0,1) = -400*x;   // derivative with respect to x and y
    res(1,1) = 200;                 // second derivative with respect to y
    return res;
}

// ----------------------------------------------------------------------------------------

class test_function
{
    /*
        This object is an example of what is known as a "function object" in C++.
        It is simply an object with an overloaded operator().  This means it can 
        be used in a way that is similar to a normal C function.  The interesting
        thing about this sort of function is that it can have state.  
        
        In this example, our test_function object contains a column_vector 
        as its state and it computes the mean squared error between this 
        stored column_vector and the arguments to its operator() function.

        This is a very simple function, however, in general you could compute
        any function you wanted here.  An example of a typical use would be 
        to find the parameters of some regression function that minimized 
        the mean squared error on a set of data.  In this case the arguments
        to the operator() function would be the parameters of your regression
        function.  You would loop over all your data samples and compute the output 
        of the regression function for each data sample given the parameters and 
        return a measure of the total error.   The dlib optimization functions 
        could then be used to find the parameters that minimized the error.
    */
public:

    test_function (
        const column_vector& input
    )
    {
        target = input;
    }

    double operator() ( const column_vector& arg) const
    {
        // return the mean squared error between the target vector and the input vector
        return mean(squared(target-arg));
    }

private:
    column_vector target;
};

// ----------------------------------------------------------------------------------------

class rosen_model 
{
    /*!
        This object is a "function model" which can be used with the
        find_min_trust_region() routine.  
    !*/

public:
    typedef ::column_vector column_vector;
    typedef matrix<double> general_matrix;

    double operator() (
        const column_vector& x
    ) const { return rosen(x); }

    void get_derivative_and_hessian (
        const column_vector& x,
        column_vector& der,
        general_matrix& hess
    ) const
    {
        der = rosen_derivative(x);
        hess = rosen_hessian(x);
    }
};

// TODO: This is very messy. My goal here is to test this out.
// A) I need to use some sort of Vector.  This system uses column_vector;
// I hardely know how to use that.
// B) Maybe I should first test multiple variables in a simpler system first!
// Can I solve 100 variables?
void print_vec(const column_vector& vec)
{
  for(int i = 0; i < vec.size(); i++) {
    std::cout << ' ' << vec(i);
    }
    std::cout << '\n';
}


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

#define LADDER_NODES 13
// #define VAR_EDGES (LADDER_NODES-3)*2+2;
#define VAR_EDGES 22
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
    double mx = 3.0;
    double my = 3.0;
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

int main()
{
    try
    {
        // // make a column vector of length 2
        // column_vector starting_point(2);


        // // Set the starting point to (4,8).  This is the point the optimization algorithm
        // // will start out from and it will move it closer and closer to the function's 
        // // minimum point.   So generally you want to try and compute a good guess that is
        // // somewhat near the actual optimum value.
        // starting_point = 4, 8;

        // // The first example below finds the minimum of the rosen() function and uses the
        // // analytical derivative computed by rosen_derivative().  Since it is very easy to
        // // make a mistake while coding a function like rosen_derivative() it is a good idea
        // // to compare your derivative function against a numerical approximation and see if
        // // the results are similar.  If they are very different then you probably made a 
        // // mistake.  So the first thing we do is compare the results at a test point: 
        // cout << "Difference between analytic derivative and numerical approximation of derivative: " 
        //       << length(derivative(rosen)(starting_point) - rosen_derivative(starting_point)) << endl;


        // cout << "Find the minimum of the rosen function()" << endl;
        // // Now we use the find_min() function to find the minimum point.  The first argument
        // // to this routine is the search strategy we want to use.  The second argument is the 
        // // stopping strategy.  Below I'm using the objective_delta_stop_strategy which just 
        // // says that the search should stop when the change in the function being optimized 
        // // is small enough.

        // // The other arguments to find_min() are the function to be minimized, its derivative, 
        // // then the starting point, and the last is an acceptable minimum value of the rosen() 
        // // function.  That is, if the algorithm finds any inputs to rosen() that gives an output 
        // // value <= -1 then it will stop immediately.  Usually you supply a number smaller than 
        // // the actual global minimum.  So since the smallest output of the rosen function is 0 
        // // we just put -1 here which effectively causes this last argument to be disregarded.

        // find_min(bfgs_search_strategy(),  // Use BFGS search algorithm
        //          objective_delta_stop_strategy(1e-7), // Stop when the change in rosen() is less than 1e-7
        //          rosen, rosen_derivative, starting_point, -1);
        // // Once the function ends the starting_point vector will contain the optimum point 
        // // of (1,1).
        // cout << "rosen solution:\n" << starting_point << endl;


        // // Now let's try doing it again with a different starting point and the version
        // // of find_min() that doesn't require you to supply a derivative function.  
        // // This version will compute a numerical approximation of the derivative since 
        // // we didn't supply one to it.
        // starting_point = -94, 5.2;
        // find_min_using_approximate_derivatives(bfgs_search_strategy(),
        //                                        objective_delta_stop_strategy(1e-7),
        //                                        rosen, starting_point, -1);
        // // Again the correct minimum point is found and stored in starting_point
        // cout << "rosen solution:\n" << starting_point << endl;


        // // Here we repeat the same thing as above but this time using the L-BFGS 
        // // algorithm.  L-BFGS is very similar to the BFGS algorithm, however, BFGS 
        // // uses O(N^2) memory where N is the size of the starting_point vector.  
        // // The L-BFGS algorithm however uses only O(N) memory.  So if you have a 
        // // function of a huge number of variables the L-BFGS algorithm is probably 
        // // a better choice.
        // starting_point = 0.8, 1.3;
        // find_min(lbfgs_search_strategy(10),  // The 10 here is basically a measure of how much memory L-BFGS will use.
        //          objective_delta_stop_strategy(1e-7).be_verbose(),  // Adding be_verbose() causes a message to be 
        //                                                             // printed for each iteration of optimization.
        //          rosen, rosen_derivative, starting_point, -1);

        // cout << endl << "rosen solution: \n" << starting_point << endl;

        // starting_point = -94, 5.2;
        // find_min_using_approximate_derivatives(lbfgs_search_strategy(10),
        //                                        objective_delta_stop_strategy(1e-7),
        //                                        rosen, starting_point, -1);
        // cout << "rosen solution: \n"<< starting_point << endl;




        // // dlib also supports solving functions subject to bounds constraints on
        // // the variables.  So for example, if you wanted to find the minimizer
        // // of the rosen function where both input variables were in the range
        // // 0.1 to 0.8 you would do it like this:
        // starting_point = 0.1, 0.1; // Start with a valid point inside the constraint box.
        // find_min_box_constrained(lbfgs_search_strategy(10),  
        //                          objective_delta_stop_strategy(1e-9),  
        //                          rosen, rosen_derivative, starting_point, 0.1, 0.8);
        // // Here we put the same [0.1 0.8] range constraint on each variable, however, you
        // // can put different bounds on each variable by passing in column vectors of
        // // constraints for the last two arguments rather than scalars.  

        // cout << endl << "constrained rosen solution: \n" << starting_point << endl;

        // // You can also use an approximate derivative like so:
        // starting_point = 0.1, 0.1; 
        // find_min_box_constrained(bfgs_search_strategy(),  
        //                          objective_delta_stop_strategy(1e-9),  
        //                          rosen, derivative(rosen), starting_point, 0.1, 0.8);
        // cout << endl << "constrained rosen solution: \n" << starting_point << endl;




        // // In many cases, it is useful if we also provide second derivative information
        // // to the optimizers.  Two examples of how we can do that are shown below.  
        // starting_point = 0.8, 1.3;
        // find_min(newton_search_strategy(rosen_hessian),
        //          objective_delta_stop_strategy(1e-7),
        //          rosen,
        //          rosen_derivative,
        //          starting_point,
        //          -1);
        // cout << "rosen solution: \n"<< starting_point << endl;

        // // We can also use find_min_trust_region(), which is also a method which uses
        // // second derivatives.  For some kinds of non-convex function it may be more
        // // reliable than using a newton_search_strategy with find_min().
        // starting_point = 0.8, 1.3;
        // find_min_trust_region(objective_delta_stop_strategy(1e-7),
        //     rosen_model(), 
        //     starting_point, 
        //     10 // initial trust region radius
        // );
        // cout << "rosen solution: \n"<< starting_point << endl;




        // // Now let's look at using the test_function object with the optimization 
        // // functions.  
        // cout << "\nFind the minimum of the test_function" << endl;

        // // column_vector target(4);
        // // starting_point.set_size(4);

        // // // This variable will be used as the target of the test_function.   So,
        // // // our simple test_function object will have a global minimum at the
        // // // point given by the target.  We will then use the optimization 
        // // // routines to find this minimum value.
        // // target = 3, 5, 1, 7;

        // // // set the starting point far from the global minimum
        // // starting_point = 1,2,3,4;
        // // find_min_using_approximate_derivatives(bfgs_search_strategy(),
        // //                                        objective_delta_stop_strategy(1e-7),
        // //                                        test_function(target), starting_point, -1);
        // // // At this point the correct value of (3,5,1,7) should be found and stored in starting_point
        // // cout << "test_function solution:\n" << starting_point << endl;

        // // // Now let's try it again with the conjugate gradient algorithm.
        // // starting_point = -4,5,99,3;
        // // find_min_using_approximate_derivatives(cg_search_strategy(),
        // //                                        objective_delta_stop_strategy(1e-7),
        // //                                        test_function(target), starting_point, -1);
        // // cout << "test_function solution:\n" << starting_point << endl;



        // // // Finally, let's try the BOBYQA algorithm.  This is a technique specially
        // // // designed to minimize a function in the absence of derivative information.  
        // // // Generally speaking, it is the method of choice if derivatives are not available.
        // // starting_point = -4,5,99,3;
        // // find_min_bobyqa(test_function(target), 
        // //                 starting_point, 
        // //                 9,    // number of interpolation points
        // //                 uniform_matrix<double>(4,1, -1e100),  // lower bound constraint
        // //                 uniform_matrix<double>(4,1, 1e100),   // upper bound constraint
        // //                 10,    // initial trust region radius
        // //                 1e-6,  // stopping trust region radius
        // //                 100    // max number of objective function evaluations
        // // );
        // // cout << "test_function solution:\n" << starting_point << endl;

	// int n = 4;
        // column_vector target(n);
        // starting_point.set_size(n);

        // // This variable will be used as the target of the test_function.   So,
        // // our simple test_function object will have a global minimum at the
        // // point given by the target.  We will then use the optimization 
        // // routines to find this minimum value.
	// //        target = 3, 5, 1, 7;
	
        // // set the starting point far from the global minimum
	// //        starting_point = 1,2,3,4;
	
	// for(int i = 0; i < n; i++) {
	//   target(i) = i;
	//   starting_point(i) = i % 8;
	// }

        // find_min_using_approximate_derivatives(bfgs_search_strategy(),
        //                                        objective_delta_stop_strategy(1e-7),
        //                                        test_function(target), starting_point, -1);
        // // At this point the correct value of (3,5,1,7) should be found and stored in starting_point
        // cout << "test_function solution:\n" << starting_point << endl;

        // // Now let's try it again with the conjugate gradient algorithm.
	// //        starting_point = -4,5,99,3;
        // find_min_using_approximate_derivatives(cg_search_strategy(),
        //                                        objective_delta_stop_strategy(1e-7),
        //                                        test_function(target), starting_point, -1);
        // cout << "test_function solution:\n" << starting_point << endl;



        // // Finally, let's try the BOBYQA algorithm.  This is a technique specially
        // // designed to minimize a function in the absence of derivative information.  
        // // Generally speaking, it is the method of choice if derivatives are not available.
	// //        starting_point = -4,5,99,3;

	// for(int i = 0; i < n; i++) {
	//   target(i) = i % 7;
	//   starting_point(i) = i % 3;
	// }

        // cout << "starting :\n" << starting_point << endl;	

	// cout <<  "UNIFORM_MATRIX" <<  uniform_matrix<double>(n,1, 12) << "\n";
        // find_min_bobyqa(test_function(target), 
        //                 starting_point, 
        //                 (n+1)*(n+2)/2,    // number of interpolation points
        //                 uniform_matrix<double>(n,1, -10),  // lower bound constraint
        //                 uniform_matrix<double>(n,1, 3),   // upper bound constraint
        //                 5,    // initial trust region radius (rho_begin)
        //                 1e-6,  // stopping trust region radius (rho_end)
        //                 1000    // max number of objective function evaluations
        // );
        // cout << "bobyqa solution:\n" << starting_point << endl;

	{
	  TriLadder an = TriLadder();

	  for (int i = 0; i < an.num_edges; ++i) {
	    an.distance(i) = MEDIAN;
	  }

	  // WARNING: We are trying here to skip over fixed edge; this should really be computed!
	  for (int i = 1; i < an.num_edges; ++i) {
	    an.distance(i) = INITIAL;
	  }

	  column_vector coords[an.num_nodes];
	  
	  find_all_coords(&an,coords);
	  solve_inverse_problem(&an);

	  for (int i = 0; i < an.num_edges; ++i) {
	    std::cout << i << " : " << an.distance(i) << std::endl;
	  }

	  find_all_coords(&an,coords);
	  Invert inv;
	  inv.an = &an;
	  column_vector sp(an.var_edges);
	  for (int i = 0; i < an.var_edges; i++) {
	    sp(i) = an.distance(i + 1);
	  }	  
	  double final = inv(sp);
	  std::cout << "inv(x) final : " << final << std::endl;
	  for(int i = 0; i < an.num_nodes; i++) {
	    std::cout << " d["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
	  }
	  
	  
	}
	
    }
    catch (std::exception& e)
    {
        cout << e.what() << endl;
    }
}

