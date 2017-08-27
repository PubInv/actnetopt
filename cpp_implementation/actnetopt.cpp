#include <iostream>
#include "../CppNumericalSolvers/include/cppoptlib/meta.h"
#include "../CppNumericalSolvers/include/cppoptlib/problem.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/bfgssolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/conjugatedgradientdescentsolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/newtondescentsolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/neldermeadsolver.h"
#include "../CppNumericalSolvers/include/cppoptlib/solver/lbfgssolver.h"
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


class ActNet {
public:

  // I know this is poor C++, I am not a very good C++ coder
  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
  // Make convenient labels for the vertices
  enum { A, B, C, D, E, F };
  const int num_vertices = 6;
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

  // This is 
  Eigen::VectorXd coords[6];
  
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
    
    const int num_edges = sizeof(edge_array)/sizeof(edge_array[0]);
    
    // declare a graph object
    Graph gg(num_vertices);
    g = gg;

    // add the edges to the graph object
    for (int i = 0; i < num_edges; ++i)
      add_edge(edge_array[i].first, edge_array[i].second, g);

    index = get(vertex_index, g);
  }
};
  
// we define a new problem for optimizing the rosenbrock function
// we use a templated-class rather than "auto"-lambda function for a clean architecture
template<typename T>
class FindCoords : public cppoptlib::Problem<T> {
public:
  using typename cppoptlib::Problem<T>::TVector;
  using typename cppoptlib::Problem<T>::THessian;
  
  Eigen::VectorXd a;
  Eigen::VectorXd b;
  //     const double dab = 1.5; // This is in fact a constant in our frame
  const double dac = 1.5; // these are in fact inputs to the problem
  const double dbc = 1.5;
  const double dcd = 1.5;
  const double dbd = 1.5;
  
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
    // We actually need to add some sort of "being on the right side test here".
    return
      pow(ac.norm() - dac,2) +
      pow(bc.norm() - dbc,2);
  }
};

// This is an attempt to use FindCoords in the correct order to find all of the things
void find_all_coords(ActNet an) {

    // initialize the Rosenbrock-problem
    typedef FindCoords<double> TFindCoords;
    TFindCoords f;

    Eigen::VectorXd temp0(2); temp0 << 0, 0;
    an.coords[0] = temp0;

    Eigen::VectorXd temp1(2); temp1 << 0, 1.5;
    an.coords[1] = temp1;

    Eigen::VectorXd a(2);
    f.a = a;
    f.a << 0, 0;
    an.coords[0][0] = f.a[0];
    an.coords[0][1] = f.a[1];    
    
    Eigen::VectorXd b(2);
    f.b = b;
    f.b << 0, 1.5;
    an.coords[1][0] = f.b[0];
    an.coords[1][1] = f.b[1];    
    
    // Basic structure: Iteratively find coordinates based on simple triangulations.
    // This only works for actuator networks in which we can
    int fs = an.fixed_nodes.size();
    for(int i = fs; i < an.num_vertices; i++) {
      // Let's set up initial values
      // choose a starting point
      Eigen::VectorXd x(2); x << 1, 2;

      cppoptlib::LbfgsSolver<TFindCoords> solver;

      f.a[0] = an.coords[i-fs][0];
      f.a[1] = an.coords[i-fs][1];      
      f.b[0] = an.coords[i-(fs-1)][0];
      f.b[1] = an.coords[i-(fs-1)][1];      

      // and minimize the function
      std::cout << "f.a " << f.a << std::endl;
      std::cout << "f.b " << f.b << std::endl;      
      
      solver.minimize(f, x);
      an.coords[i][0] = x[0];
      an.coords[i][1] = x[1];

      // print argmin
      // Now take x and make it the first coord...
      
      std::cout << "argmin " << x.transpose() << std::endl;
      std::cout << "f(x) " << f(x) << std::endl;
      
      std::cout << "node processed" << i << std::endl;
    }
};


int main(int argc, char const *argv[]) {
    ActNet an = ActNet();
    std::cout << "vertices(g) = ";

    std::pair<ActNet::vertex_iter, ActNet::vertex_iter> vp;
    for (vp = vertices(an.g); vp.first != vp.second; ++vp.first)
      std::cout << an.index[*vp.first] <<  " ";
    std::cout << std::endl;

    for (int i = 0; i < an.num_vertices; i++) {
      Eigen::VectorXd a(2);
      an.coords[i] = a;      
      an.coords[i] << 0, 0;      
    }

    find_all_coords(an);
}
