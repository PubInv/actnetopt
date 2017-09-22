#ifndef TRILADDER_H
#define TRILADDER_H 1

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
void print_vec(column_vector& vec);


const bool debug_find = false;
const bool debug = false;

double distance(column_vector a, column_vector b);

double l2_norm(column_vector a);

class TriLadder {
public:

  // I know this is poor C++, I am not a very good C++ coder
  //  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
  // Make convenient labels for the vertices
  enum { A, B, C, D, E, F };
  // Making this a const seems to destry to the implicit
  // copy assignment; I have no idea why


  int num_nodes;   // = LADDER_NODES;
  int num_edges; //  = (num_nodes-3)*2 + 3;
  int var_edges;   // = num_edges-1;
  double upper_bound_d;
  double lower_bound_d;
  double median_d;
  double initial_d;

  int *node_fixing_order;
  column_vector*  coords;

  int edges_in_ladder(int n);  

  TriLadder(int nodes,
	    double u,
	    double l,
	    double m,
	    double i);
  
  const char* name = "ABCDE";

  // writing out the edges in the graph
  //  typedef std::pair<int, int> Edge;

  //  Graph g;
  //  Edge e;

  //  static const int num_edges = 3;  
  //  Edge edge_array[num_edges];
  column_vector fixed_nodes;
  //  int node_fixing_order[num_nodes];
  //  typedef property_map<Graph, vertex_index_t>::type IndexMap;
  //  typedef graph_traits<Graph>::vertex_iterator vertex_iter;
  //  IndexMap index;

  column_vector goals[1];

  column_vector distance;
  
  column_vector lower_bound;
  column_vector upper_bound;
  
  //  Eigen::VectorXd lower_bound;
  //  Eigen::VectorXd upper_bound;  

  // This is a map into the goal position for each goal.
  // goal_nodes[a] = b => goals[a] should be considered node b.
  std::vector<int> goal_nodes;


  double gscore();
  double lscore();
  // This really assumes a single goal node is being moved;
  // in order for this to work for many goal nodes I would have
  // to accept a set of labeled vectors for each goal node.
 double compute_d_score_from_vector(double x,double y);   


// The goal of this is to compute a single derivative vector for the
// change in length to edge number edge_number.
// This returns a simple double because we have made the penalty function a
// part of it.  
void compute_single_derivative(column_vector cur_coords[],
			       int edge_number,
			       double *x,
			       double *y);
 
// I think this is the form actually required by functions such as find_min_box_constrained
// Probably this will have to call compute_single_derivative for each edge.
column_vector derivative (const column_vector& m);


  
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
  double operator() ( const column_vector& x) const;
};

// This is really a determinant of a 3x3 matrix with a column on the right
double sense(double a, double b, double d, double e, double g, double h);


// This is an attempt to use FindCoords in the correct order to find all of the things

const bool debug_find_third = false;
// This this so that it doesn't modify *an but rather returns the coordinate vector
column_vector find_third_point_given_two_and_distances(Chirality sense,double dab,double dbc,double dca,
						       column_vector pa,column_vector pb);

const double SMALL_DEFINITION = 1.0e-3;
bool any_too_small(double a, double b, double c);

void fix_sense(bool desired_sense,double x0,double y0,double x1,double y1,column_vector z);

void find_all_coords(TriLadder *an,column_vector coords[]);

void   test_find_third_point_given_two_and_distances();

column_vector change_in_third_point(column_vector a,
				    column_vector b,
				    column_vector c,
				    column_vector da,
				    column_vector db
				    );


#endif
