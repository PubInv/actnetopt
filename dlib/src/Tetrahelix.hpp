// Tetrahelix.hpp -- Header for an optimizable tetrahelix manipulator
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


#ifndef TETRAHELIX_HPP
#define TETRAHELIX_HPP 1

#include "ActNetUtility.hpp"


class Tetrahelix {
public:

  // I know this is poor C++, I am not a very good C++ coder
  //  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
  // Make convenient labels for the vertices
  enum { A, B};
  // Making this a const seems to destry to the implicit
  // copy assignment; I have no idea why

  int num_nodes;   // 
  int num_edges; //  f(num_nodes)
  int var_edges;   // = num_edges-1;
  double upper_bound_d;
  double lower_bound_d;
  double median_d;
  double initial_d;

  int *node_fixing_order;
  column_vector*  coords;

  int edges_in_tetrahelix(int n);

  int large_node(int e);
  int small_node(int e);
  int edge_between(int ndh,int ndl);

  void add_goal_node(int num, double x,double y, double w);

  Tetrahelix(int nodes,
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

  //  column_vector goals[1];
  std::vector<column_vector> goals;


  column_vector distance;
  
  column_vector lower_bound;
  column_vector upper_bound;
  
  // This is a map into the goal position for each goal.
  // goal_nodes[a] = b => goals[a] should be considered node b.
  std::vector<int> goal_nodes;
  std::vector<double> goal_weights;

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
double compute_single_derivative_dtheta(column_vector cur_coords[],
			       int edge_number);

column_vector compute_single_derivative_c(column_vector cur_coords[],
				   int edge_number);


double compute_dtheta_internal_da(column_vector A,
			 column_vector B,
			 column_vector C,
			 column_vector D,
			 double a,
			 double b,
			 double c,
			 double f,
				     double g);

column_vector compute_goal_derivative_c(column_vector cur_coords[],
								int edge_number,
						   int goal_node_number);
 
column_vector compute_external_effector_derivative_c(column_vector cur_coords[],
						     int edge_number,
						     int goal_node_number);

column_vector compute_internal_effector_derivative_c(column_vector cur_coords[],
						      int edge_number,
						      int goal_node_number);
 
 void set_fixed_coords(column_vector coords[]);

};


#endif
