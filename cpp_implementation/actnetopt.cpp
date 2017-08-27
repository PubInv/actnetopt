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

class ActNet {
public:
  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
  // Make convenient labels for the vertices
  enum { A, B, C, D, E, N };
  const int num_vertices = N;
  const char* name = "ABCDE";

  // writing out the edges in the graph
  typedef std::pair<int, int> Edge;

  Graph g;
  Edge e;
  Edge edge_array[7];  
  
  typedef property_map<Graph, vertex_index_t>::type IndexMap;
  
  typedef graph_traits<Graph>::vertex_iterator vertex_iter;
  
  ActNet() {
   // Edge ea[] = 
   //    { Edge(A,B), Edge(A,D), Edge(C,A), Edge(D,C),
   // 	Edge(C,E), Edge(B,D), Edge(D,E) };
   edge_array[0] = Edge(A,B);
    
    const int num_edges = sizeof(edge_array)/sizeof(edge_array[0]);

    // declare a graph object
    Graph gg(num_vertices);
    g = gg;

    // add the edges to the graph object
    for (int i = 0; i < num_edges; ++i)
      add_edge(edge_array[i].first, edge_array[i].second, g);


    IndexMap index = get(vertex_index, g);
    
    typedef property_map<Graph, vertex_index_t>::type IndexMap;
    
    std::cout << "vertices(g) = ";
    typedef graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(g); vp.first != vp.second; ++vp.first)
      std::cout << index[*vp.first] <<  " ";
    std::cout << std::endl;
  }
  
};
  
int pretend_to_create_a_graph()
{
  // create a typedef for the Graph type
  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;

  // Make convenient labels for the vertices
  enum { A, B, C, D, E, N };
  const int num_vertices = N;
  const char* name = "ABCDE";

  // writing out the edges in the graph
  typedef std::pair<int, int> Edge;
  Edge edge_array[] = 
    { Edge(A,B), Edge(A,D), Edge(C,A), Edge(D,C),
      Edge(C,E), Edge(B,D), Edge(D,E) };
  const int num_edges = sizeof(edge_array)/sizeof(edge_array[0]);

  // declare a graph object
  Graph g(num_vertices);

  // add the edges to the graph object
  for (int i = 0; i < num_edges; ++i)
    add_edge(edge_array[i].first, edge_array[i].second, g);

  typedef property_map<Graph, vertex_index_t>::type IndexMap;
  IndexMap index = get(vertex_index, g);

  std::cout << "vertices(g) = ";
  typedef graph_traits<Graph>::vertex_iterator vertex_iter;
  std::pair<vertex_iter, vertex_iter> vp;
  for (vp = vertices(g); vp.first != vp.second; ++vp.first)
    std::cout << index[*vp.first] <<  " ";
  std::cout << std::endl;
  return 0;
}


// we define a new problem for optimizing the rosenbrock function
// we use a templated-class rather than "auto"-lambda function for a clean architecture
template<typename T>
class FindCoords : public cppoptlib::Problem<T> {
  public:
    using typename cppoptlib::Problem<T>::TVector;
    using typename cppoptlib::Problem<T>::THessian;

    // this is just the objective (NOT optional)
  // This input is an x,y position for c
  // The lengths between a, b, and c are constants (effectively, input to the problem)
    T value(const TVector &x) {

      Eigen::VectorXd a(2); a << 0, 0;
      Eigen::VectorXd b(2); b << 0, 1.5;
      //     const double dab = 1.5; // This is in fact a constant in our frame
      const double dac = 1.5; // these are in fact inputs to the problem
      const double dbc = 1.5;
      const double dcd = 1.5;
      const double dbd = 1.5;

      Eigen::VectorXd y(2); y  << x[0], x[1];
      Eigen::VectorXd z(2); z  << x[2], x[3];      

      Eigen::VectorXd bc(2);
      bc = y - b;
      Eigen::VectorXd ac(2);
      ac = y - a;
      Eigen::VectorXd bd(2);
      bd = b - z;
      Eigen::VectorXd cd(2);
      cd = y - z;
      return
	pow(ac.norm() - dac,2) +
	pow(bc.norm() - dbc,2) +
	pow(bd.norm() - dbd,2) +
	pow(cd.norm() - dcd,2);
    }

};

int main(int argc, char const *argv[]) {

    // initialize the Rosenbrock-problem
    typedef FindCoords<double> TFindCoords;
    TFindCoords f;

    ActNet an = ActNet();

    // choose a starting point
    Eigen::VectorXd x(4); x << 1, 2, 3, 4;

    // first check the given derivative 
    // there is output, if they are NOT similar to finite differences
    //    bool probably_correct = f.checkGradient(x);
    //    std::cout << "probably_correct: " << probably_correct << std::endl;
    
    // choose a solver
    //    cppoptlib::BfgsSolver<TFindCoords> solver;
    // cppoptlib::BfgsSolver<TFindCoords> solver;
    //cppoptlib::ConjugatedGradientDescentSolver<T> solver;
    //cppoptlib::NewtonDescentSolver<T> solver;
    // cppoptlib::NelderMeadSolver<TFindCoords> solver;
    cppoptlib::LbfgsSolver<TFindCoords> solver;
    // cppoptlib::CMAesSolver<TFindCoords> solver;
    // and minimize the function
    
    // and minimize the function
    solver.minimize(f, x);
    // print argmin
    std::cout << "argmin " << x.transpose() << std::endl;
    std::cout << "f(x) " << f(x) << std::endl;


    pretend_to_create_a_graph();
    
    return 0;
}
