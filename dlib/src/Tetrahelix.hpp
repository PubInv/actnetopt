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
#include "Obstacle.hpp"


class Tetrahelix {
public:

  // I know this is poor C++, I am not a very good C++ coder
  //  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
  // Make convenient labels for the vertices
  enum { A, B, C};
  // Making this a const seems to destry to the implicit
  // copy assignment; I have no idea why

  Obstacle *obstacle = NULL;  

  int num_nodes;   // 
  int num_edges; //  f(num_nodes)
  int var_edges;   // = num_edges-1;
  double upper_bound_d;
  double lower_bound_d;
  double median_d;
  double initial_d;

  int *node_fixing_order;
  column_vector*  lcoords;
  column_vector fixed[3];

  void init_fixed_coords(column_vector fcoords[]);
  void init_fixed_coords_to_z_axis_alignment(column_vector coords[]);
  void restore_fixed_coords(column_vector coords[]);  

  int edges_in_tetrahelix(int n);

  int edge_number_of_nth_variable_edge(int n);  
  int large_node(int e);
  int small_node(int e);
  int edge_between(int ndh,int ndl);
  bool simple_hinge_p(int e);
  
  int color_of_node(int n);
  bool rail_edge_p(int e);
  void add_goal_node(int num, double x,double y, double z,double w);

  Tetrahelix(int nodes,
	    double u,
	    double l,
	    double m,
	    double i);
  
  const char* name = "ABCDE";

  column_vector fixed_nodes;
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

column_vector compute_goal_differential_c(column_vector cur_coords[],
			    int edge_number,
			    int goal_node_number,
			    double delta_fraction = 0.01);  
  
column_vector compute_goal_derivative_c(column_vector cur_coords[],
								int edge_number,
						   int goal_node_number);

column_vector compute_goal_derivative_after_edge_internal(column_vector cur_coords[],
								int edge_number,
							   int goal_node_number);
  
 
column_vector compute_external_effector_derivative_c(column_vector cur_coords[],
						     int edge_number,
						     int goal_node_number);

column_vector compute_internal_effector_derivative_c(column_vector cur_coords[],
						      int edge_number,
						      int goal_node_number);
 
 void set_fixed_coords(column_vector coords[]);
  void set_distances(column_vector coords[]);  


  //  double private_f(double CD, double CE, double ED);
  //  double private_g(double CD, double BC, double BD); 
  double dcos_adj(double adj1, double adj2, double opp);
  //  double private_gdz(double CD, double BC, double BD);

// This is an preliminiary attempt to test the

  double d_dihedralBC_dCD_aux(double BC, double BD, double CD, double CE, double DE, double aBCE);
  
  double d_dihedralBC_dCD(column_vector cur_coords[],
				    int edge_number);

  matrix<double> Jacobian(column_vector coords[],int node);
  
};

class FindCoords3d {
public:
  column_vector a;
  column_vector b;
  column_vector c;      

  Tetrahelix *an;
  
  // Should the third point cc or ccw from a to b?
  // In other words, we use this to disambiguate the two distance based solutions.
  Chirality chi;

  //     const double dab = 1.5; // This is in fact a constant in our frame
  // "dad = distance from a to d "
  // "dbd = distance from b to d " 
  double dad; // these are in fact inputs to the problem
  double dbd;
  double dcd;
  
  // this is just the objective (NOT optional)
  // This input is an x,y position for c
  // The lengths between a, b, and c are constants (effectively, input to the problem)
  double operator() ( const column_vector& x) const;
};

bool solve_forward_find_coords(Tetrahelix *an,column_vector coords[]);


column_vector find_point_from_transformed(Chirality sense,double AB, double AC, double AD, double BC, double BD, double CD,bool *valid);

  
column_vector find_fourth_point_given_three_points_and_three_distances(
								      Chirality sense,
								      column_vector pa,column_vector pb,column_vector pc,
								      double da,double db,double dc,bool* valid
								      );

point_transform_affine3d compute_transform_to_axes(column_vector A, column_vector B, column_vector C);

point_transform_affine3d compute_transform_to_axes2(column_vector A, column_vector B, column_vector C);


double dtheta_dd_laid_on_axes(double d, double x, double z);

// In this case, x,y,z are the lengths of a triangle
double darea_dx(double x,double y, double z);

// a,b,c,d are the areas of the triangles opposite i,j,k,l respectively.
// e,f,g and dihedral angles of il,ik, and ij respectively.
// our basic goal is to compute dihedral angles; this is part of that.
double dangle_db(double a, double b, double c, double d, double e, double f);

double semi_perimeter(double x, double y, double z);

double heron_area(double x,double y, double z);

// find the dihedral angle of <AB given vertex angles...
double dihedral_from_vertex_angles(double a, double b, double c);

double angle_from_three_sides(double adjacent1, double adjacent2, double opposite);

double dangle_from_dside(double adjacent1, double adjacent2, double opposite);

double ddhedral_dvertex(double adjacent1, double adjacent2, double opposite);

column_vector compute_rotation_about_points(column_vector A,
					    column_vector B,
					    double theta,
					    column_vector M);



#endif
