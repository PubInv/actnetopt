#include <iostream>
#include "../CppNumericalSolvers/include/cppoptlib/meta.h"
#include "../CppNumericalSolvers/include/cppoptlib/problem.h"
#include "../CppNumericalSolvers/include/cppoptlib/boundedproblem.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/bfgssolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/conjugatedgradientdescentsolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/newtondescentsolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/neldermeadsolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/lbfgssolver.h"
// #include "../CppNumericalSolvers/include/cppoptlib/solver/lbfgsbsolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/cmaessolver.h"
#include <cmath>

#include <cstddef>
#include <stdexcept>

#include <boost/lambda/lambda.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace boost;
using namespace cppoptlib;

// Okay, we now attempt to develope a model.

// These are a number of ways of creating solvers.
// choose a solver
//    cppoptlib::BfgsSolver<TFindCoords> solver;
//  cppoptlib::BfgsSolver<TFindCoords> solver;
//  cppoptlib::ConjugatedGradientDescentSolver<T> solver;
//  cppoptlib::NewtonDescentSolver<T> solver;
//  cppoptlib::NelderMeadSolver<TFindCoords> solver;
//    cppoptlib::LbfgsSolver<TFindCoords> solver;
// cppoptlib::CMAesSolver<TFindCoords> solver;

const bool debug_find = false;
const bool debug = false;


class ActNet {
public:

  // I know this is poor C++, I am not a very good C++ coder
  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
  // Make convenient labels for the vertices
  enum { A, B, C, D, E, F };
  // Making this a const seems to destry to the implicit
  // copy assignment; I have no idea why
  int num_vertices = 6;
  const char* name = "ABCDE";

  // writing out the edges in the graph
  typedef std::pair<int, int> Edge;

  Graph g;
  Edge e;
  Edge edge_array[7];
  std::vector<int> fixed_nodes;
  int node_fixing_order[6];
  typedef property_map<Graph, vertex_index_t>::type IndexMap;
  typedef graph_traits<Graph>::vertex_iterator vertex_iter;
  IndexMap index;
  Eigen::VectorXd coords[6];
  Eigen::VectorXd goals[1];

  std::vector<double> distance;
  
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
  
  //  Eigen::VectorXd lower_bound;
  //  Eigen::VectorXd upper_bound;  

  // This is a map into the goal position for each goal.
  // goal_nodes[a] = b => goals[a] should be considered node b.
  std::vector<int> goal_nodes;
  int num_edges;
  ActNet() {

    edge_array[0] = Edge(A,B);
    edge_array[1] = Edge(B,C);
    edge_array[2] = Edge(A,C);      
    edge_array[3] = Edge(D,C);
    edge_array[4] = Edge(C,E);
    edge_array[5] = Edge(B,D);
    edge_array[6] = Edge(D,E);

    fixed_nodes.push_back(A);
    fixed_nodes.push_back(B);

    node_fixing_order[0] = A;
    node_fixing_order[1] = B;
    node_fixing_order[2] = C;
    node_fixing_order[3] = D;
    node_fixing_order[4] = E;
    node_fixing_order[5] = F;
    
    num_edges = sizeof(edge_array)/sizeof(edge_array[0]);
    
    // declare a graph object
    Graph gg(num_vertices);
    g = gg;

    // add the edges to the graph object
    for (int i = 0; i < num_edges; ++i) {
      add_edge(edge_array[i].first, edge_array[i].second, g);
      //      lower_bound[i] = 1.2;
      //      upper_bound[i] = 2.0;
      lower_bound.push_back(1.2);
      lower_bound.push_back(2.0);
      // This is just a beginning...
      distance.push_back(1.5);
    }

    Eigen::VectorXd gl(2); gl << 3.0, 4.0;
    goals[0] = gl;

    goal_nodes.push_back(5);
    
    index = get(vertex_index, g);
  }
};

typedef enum { CW, CCW } Chirality;

// we define a new problem for optimizing the rosenbrock function
// we use a templated-class rather than "auto"-lambda function for a clean architecture
template<typename T>
class FindCoords : public cppoptlib::Problem<T> {
public:
  using typename cppoptlib::Problem<T>::TVector;
  using typename cppoptlib::Problem<T>::THessian;
  
  Eigen::VectorXd a;
  Eigen::VectorXd b;

  ActNet *an;

  
  // Should the third point cc or ccw from a to b?
  // In other words, we use this to disambiguate the two distance based solutions.
  Chirality chi;

  //     const double dab = 1.5; // This is in fact a constant in our frame
  double dac = 1.5; // these are in fact inputs to the problem
  double dbc = 1.5;
  
  // this is just the objective (NOT optional)
  // This input is an x,y position for c
  // The lengths between a, b, and c are constants (effectively, input to the problem)
  T value(const TVector &x) {
    // Eigen::VectorXd aa(2);
    // a = aa;
    
    // Eigen::VectorXd bb(2);
    // b = bb;
    
    Eigen::VectorXd y(2); y  << x[0], x[1];

    Eigen::VectorXd bc(2);
    bc = y - b;
    Eigen::VectorXd ac(2);
    ac = y - a;


    return
      pow(ac.norm() - dac,2) +
      pow(bc.norm() - dbc,2);
  }
};

// This is an attempt to use FindCoords in the correct order to find all of the things
void find_all_coords(ActNet *an) {

    // initialize the Rosenbrock-problem
    typedef FindCoords<double> TFindCoords;
    TFindCoords f;



    Eigen::VectorXd temp0(2); temp0 << 0, 0;
    an->coords[0] = temp0;

    Eigen::VectorXd temp1(2); temp1 << 0, 1.5;
    an->coords[1] = temp1;

    Eigen::VectorXd a(2);
    f.a = a;
    f.a << 0, 0;
    an->coords[0][0] = f.a[0];
    an->coords[0][1] = f.a[1];    
    
    Eigen::VectorXd b(2);
    f.b = b;
    f.b << 0, 1.5;
    an->coords[1][0] = f.b[0];
    an->coords[1][1] = f.b[1];    
    
    // Basic structure: Iteratively find coordinates based on simple triangulations.
    // This only works for actuator networks in which we can
    int fs = an->fixed_nodes.size();
    for(int i = fs; i < an->num_vertices; i++) {
      // Let's set up initial values
      // choose a starting point
      Eigen::VectorXd x(2); x << 1, 2;

      cppoptlib::LbfgsSolver<TFindCoords> solver;

      f.a[0] = an->coords[i-fs][0];
      f.a[1] = an->coords[i-fs][1];      
      f.b[0] = an->coords[i-(fs-1)][0];
      f.b[1] = an->coords[i-(fs-1)][1];

      Chirality desired_sense = ((i % 2) == 0) ? CW : CCW;
      f.chi = desired_sense;

      // and minimize the function
      if (debug_find) std::cout << "f.a " << f.a << std::endl;
      if (debug_find) std::cout << "f.b " << f.b << std::endl;


      f.dac = an->distance[i-fs];
      f.dbc = an->distance[i-(fs-1)];
      
      solver.minimize(f, x);

      // If x is not in the correct clock-sense, then we need to reflect it across
      // the vector a,b:
      // https://stackoverflow.com/questions/3306838/algorithm-for-reflecting-a-point-across-a-line

      // https://math.stackexchange.com/questions/1324179/how-to-tell-if-3-connected-points-are-connected-clockwise-or-counter-clockwise
      Eigen::Matrix3d ma;
      ma << f.a[0], f.a[1], 1,
	f.b[0], f.b[1], 1,
	x[0], x[1], 1;

      double det = ma.determinant();
      // d > 0 => ccw, d < 0 => cw.
      // now how do we make this penalize for going the wrong way?
      bool found_sense = (det > 0) ? CCW : CW;

      if (debug_find) std::cout << "processing node: " << i << std::endl;
      
      if (debug_find) std::cout << "det " << det << std::endl;
      
      bool need_to_flip = (desired_sense != found_sense);

      if (need_to_flip) {
	if (debug_find) std::cout << "NEED TO FLIP" << std::endl;
	if (debug_find) std::cout << "start: " << x.transpose() << std::endl;
	Eigen::VectorXd r(2);
      
	// first we obtain the y = mx + b form..
	if ( (f.a[0]-f.b[0]) == 0) {
	  if (debug_find) std::cout << " SLOPE ZERO "  << std::endl;	  
	  // In this case, we have a vertical line, we can tell based on
	  // just where it is based on the x value of x and whether a is above
	  r << 2*f.a[0] - x[0],x[1];
	} else {
	  const double m = (f.a[1]-f.b[1]) / (f.a[0]-f.b[0]);
	  const double b = f.a[1] - m * f.a[0];
	  const double d = (x[0] + (x[1] - b)*m)/(1+m*m);
	  const double xp = 2*d - x[0];
	  const double yp = 2*d*m - x[1] + 2 * b;
	  r << xp, yp;
	}
	if (debug_find) std::cout << "flipped: " <<  r.transpose()  << std::endl;
	x[0] = r[0];
	x[1] = r[1];
      }
      if (debug_find) std::cout << " x " << x.transpose() << std::endl;

      an->coords[i][0] = x[0];
      an->coords[i][1] = x[1];
      
      // print argmin
      // Now take x and make it the first coord...
      
      if (debug_find) std::cout << "f(x) " << f(x) << std::endl;
    }
    for (int i = 0; i < an->num_vertices; i++) {
      if (debug_find) std::cout << i << " =  " << an->coords[i] << std::endl;      
    }
    
};



// So now that we have a reasonalbe "find_coords" working,
// although it could be greatly improved, we can attempt
// to build an "inverse" routine that uses find_coords
// to try to place points in desired positions subject to
// length constraints.

void print_vec(const std::vector<double>& vec)
{
    for (auto x: vec) {
         std::cout << ' ' << x;
    }
    std::cout << '\n';
}

template<typename T>
class Invert : public cppoptlib::Problem<T> {
public:
  using typename cppoptlib::Problem<T>::TVector;
  using typename cppoptlib::Problem<T>::THessian;
						
  ActNet *an;
  Invert() {
  }
  
  // This function weights our close our values are to the goals.  
  T value(const TVector &ds) {
    // ds is a the length of each node.
    // we set those length into the ActNet...
     an->distance.clear();
     auto it = an->distance.begin();
    for (int i = 0; i < an->num_edges; ++i) {
      //      an->distance.insert(i,ds[i]);
      it = an->distance.insert(it, ds[i]);      
    }
    std::cout << "DISTANCES" << std::endl;
    print_vec(an->distance);

    // Then we find the coords...
    find_all_coords(an);

    // Loop over each goal, penalizing distance from the goal.
    // This is a only penalizing the goal.
    
    double v = 0.0;
    for(int i = 0; i < an->goal_nodes.size(); i++) {
      int idx = an->goal_nodes[i];
      if (debug) std::cout << "idx " <<  idx  << std::endl;      
      Eigen::VectorXd g = an->goals[i];
      Eigen::VectorXd c = an->coords[idx];      
      Eigen::VectorXd x(2); x << g[0], g[1];
      Eigen::VectorXd y(2); y << c[0], c[1];
      Eigen::VectorXd d(2);
      if (debug) std::cout << "Invert x " <<  x[0] <<  "," << x[1]  << std::endl;
      if (debug)std::cout << "Invert y " <<  y[0] <<  "," << y[1]  << std::endl;                  
      d = x - y;
      if (debug) std::cout << "Invert d " <<  d[0] <<  "," << d[1] <<  " " << std::endl;      
      v += pow(d.norm(),2);
    }
    if (debug) std::cout << "Invert v " <<  v <<  " " << std::endl;

    // now we also need to add in a penalty for distances being too high..
    // Possibly we can do this as a different mechanism.
    
    return v;
  }
};


void solve_inverse_problem(ActNet *an) {
  
  typedef Invert<double> TInvert;  
  TInvert inv;
  inv.an = an;

  cppoptlib::LbfgsSolver<TInvert> isolver;

  // This in fact must be the distances that we seek.
  // It must be the (unfixed) edges in the ActNet

  // This constant actually must be removed.
  Eigen::VectorXd x(7);

  // We need the ActNet to have distances in order to iniitilize this meaningfully
  for (int i = 0; i < 7; i++) {
    x[i] = an->distance[i];
  }

  //   inv.setBoxConstraint(an->lower_bound,an->upper_bound);
   
   //   inv.setLowerBound(an->lower_bound);
   //   inv.setUpperBound(an->upper_bound);
   
   isolver.minimize(inv, x);

   for (int i = 0; i < 7; i++) {
     if (debug) std::cout << "distance " << i << " :  " << x[i] << std::endl;      
   }
   if (debug) std::cout << "inv(x) " << inv(x) << std::endl;  
  
};

int main(int argc, char const *argv[]) {
    ActNet an = ActNet();
    if (debug) std::cout << "vertices(g) = ";

    std::pair<ActNet::vertex_iter, ActNet::vertex_iter> vp;
    for (vp = vertices(an.g); vp.first != vp.second; ++vp.first)
      std::cout << an.index[*vp.first] <<  " ";
    
    std::cout << std::endl;

    for (int i = 0; i < an.num_vertices; i++) {
      Eigen::VectorXd a(2);
      an.coords[i] = a;      
      an.coords[i] << 0, 0;      
    }

    // I think this is doing an implicit copy, and I'm not sure that is what we want.
    find_all_coords(&an);
    
    for (int i = 0; i < an.num_vertices; i++) {
      if (debug) std::cout << i << " =  " << an.coords[i] << std::endl;      
    }

    solve_inverse_problem(&an);
    
    for (int i = 0; i < an.num_vertices; i++) {
      std::cout << "node" << i << " =  " << an.coords[i] << std::endl;      
    }

    for (int i = 0; i < an.num_edges; ++i) {
      std::cout << "edge" << i << " =  " << an.distance[i] << std::endl;            
    }
    
}
