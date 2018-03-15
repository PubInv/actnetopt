// TriLadder.cpp -- Code to implement a "Warren" Truss or V-Truss in 2d
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


#include <stdio.h>
#include <iostream>
#include <dlib/geometry.h>

#include <math.h>

#define PI 3.14159265

#include "ActNetUtility.hpp"
#include "TriLadder.hpp"

using namespace std;
using namespace dlib;


// ----------------------------------------------------------------------------------------

// In dlib, the general purpose solvers optimize functions that take a column
// vector as input and return a double.  So here we make a typedef for a
// variable length column vector of doubles.  This is the type we will use to
// represent the input to our objective functions which we will be minimizing.
// typedef matrix<double,0,1> column_vector;


// void print_vec(column_vector& vec)
// {
//   for(int i = 0; i < vec.size(); i++) {
//     std::cout << ' ' << vec(i);
//     }
//     std::cout << '\n';
// }

// template <typename T> int sgn(T val) {
//     return (T(0) < val) - (val < T(0));
// }

// void physical_to_viewport(double px,double py,double *vx, double *vy);
// void viewport_to_physical(double px,double py,double *vx, double *vy);

// const bool debug_find = false;
// const bool debug = false;



int TriLadder::edges_in_ladder(int n) {
  return (n-3)*2 + 3;
}

int TriLadder::large_node(int e) {
  return small_node(e) + (((e % 2) == 0) ? 1 : 2);
}

int TriLadder::small_node(int e) {
  return e / 2;
}

// Compute the edge number between a higher and lower node.
int TriLadder::edge_between(int ndh,int ndl) {
  if (ndl >= ndh) {
    cout << "ndh,ndl" << ndh << " " << ndl << "\n";
    cout << "INTERNAL ERROR\n";
    abort();
  }
  int d = ndh-ndl;
  if (d == 1) {
    return (ndh * 2) - 2;    
  } else if (d == 2) {
    return (ndh * 2) - 3;
  } else {
    cout << "ndh,ndl" << ndh << " " << ndl << "\n";    
    cout << "INTERNAL ERROR\n";
    abort();
  }
}


    // Here for testing I adding a second goal of a middle node
    // and fixing it to approximately a midpoint.
void TriLadder::add_goal_node(int num,double x,double y, double w)    {
      column_vector mg(2);
      mg(0) = x;
      mg(1) = y;
      goals.push_back(mg);
      goal_nodes.push_back(num);
      goal_weights.push_back(w);
    }

// Note on edge numbering: Nodes are numbered in order of the
// so that the lowest possible two nodes first.
// So it order AB first.  Then it orders AC, since A+C < A+B.
// Then it orders AC etc.

TriLadder::TriLadder(int nodes,
		     double u,
		     double l,
		     double m,
		     double i) {
  upper_bound_d = u;
  lower_bound_d = l;
  median_d = m;
  initial_d = i;

    num_nodes = nodes;
    
    num_edges = edges_in_ladder(num_nodes);
    
    var_edges = num_edges-1;
    node_fixing_order = new int[num_nodes];
    coords = new column_vector[num_nodes];      

    fixed_nodes.set_size(2);
    fixed_nodes(0) = A;
    fixed_nodes(1) = B;

    // add the edges to the graph object
    distance.set_size(num_edges);
    lower_bound.set_size(num_edges);
    upper_bound.set_size(num_edges);    
    for (int i = 0; i < num_edges; ++i) {
      distance(i) = median_d;
    }
    
    for (int i = 0; i < var_edges; ++i) {
      lower_bound(i) = lower_bound_d;
      upper_bound(i) = upper_bound_d;            
    }
  }
  double TriLadder::gscore() {
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
      d = x - y;
      if (debug) std::cout << "Invert d " <<  d(0) <<  "," << d(1) <<  " " << std::endl;
      double vp = d(0)*d(0) + d(1)*d(1);
      v += goal_weights[i]*vp;
    }
    return v;
  }
  double TriLadder::compute_d_score_from_vector(double x,double y) {
    double v = 0.0;
    return v;
  }

double TriLadder::lscore() {
    return 0.0;
  }

column_vector normalize(column_vector v) {
  column_vector r(2);
  column_vector zero(2);
  zero(0) = 0.0;
  zero(1) = 0.0;
  double len = distance_2d(zero,v);
  r(0) = v(0)/len;
  r(1) = v(1)/len;
  return r;
}

// This is to compute the SIGNED Angle between A to B to C (move C into A).
// I think this is computing the anticlockwise angle.
double get_angle(column_vector a, column_vector b, column_vector c) {
  column_vector Va = (a-b);
  column_vector Vc = (c-b);
  Va = normalize(Va);
  Vc = normalize(Vc);
  double aangle = atan2(Va(1), Va(0));
  double cangle = atan2(Vc(1), Vc(0));
  
  double angle = aangle - cangle;

  if (angle >= PI) angle = -(2*PI - angle);
  if (angle <= -PI) angle = 2*PI + angle;

  return angle;
}


// under the current numbering, externals are odd...
bool external_edge(int edge) {
  return ((edge % 2) == 1);
}
column_vector TriLadder::compute_goal_derivative_c(column_vector cur_coords[],
								int edge_number,
								int goal_node_number) {
  return (external_edge(edge_number)) ?
	  compute_external_effector_derivative_c(cur_coords,
						 edge_number,
						 goal_node_number)
	  : compute_internal_effector_derivative_c(cur_coords,
						   edge_number,
						   goal_node_number);
}

column_vector TriLadder::compute_external_effector_derivative_c(column_vector cur_coords[],
								int edge_number,
								int goal_node_number) {
  // First, let us set up our variables...
  // This nomenclature matches the paper "OnSimplePlanarVariableGeomertyTrusses"
  // e is the end effector
  column_vector en = cur_coords[goal_node_number];
  
  double enx = en(0);
  double eny = en(1);

  // let P be the joint about which we are rotating by changing e
  // let M be the directly moved joint.
  // let S be the "stable" joint connected to M by e.

  int e = edge_number;
  int M = large_node(e);
  int S = small_node(e);
  int P = (M+S)/2;


  column_vector d_e_a(2);

  if (goal_node_number <= S) {
    d_e_a(0) = 0.0;
    d_e_a(1) = 0.0;
    return d_e_a;
  }
  
  column_vector m = cur_coords[M];
  column_vector p = cur_coords[P];
  column_vector s = cur_coords[S];  
    
  double x = cur_coords[P](0);
  double y = cur_coords[P](1);
  
  double a = distance_2d(s,p);
  double b = distance_2d(s,m);
  double c = distance_2d(p,m);

  // f is just a name for "factor"...
  // f0 is just a placeholder
  // Unfortunately, we need to compute the sign of this...
  // We should be able to do this based on the sign of < SPM
  double spm = -get_angle(s,p,m);

  double f0 = (a*a + c*c - b*b)/ (4*a*a * c*c);
  double f = b / (a*c*sqrt(1 - f0));

  f = f * sgn(spm);

  // theta is angle about P of S into M
  // I worked the math in that paper out based on anticlockwise angles!
  //  double theta = -get_angle(s,p,m);
  column_vector xaxis(2);
  xaxis(0) = 10.0;
  xaxis(1) = 0.0;

  column_vector pivot_to_en = en - p;

  double xd = (enx - x);
  double yd = (eny - y);

  // d_e_a is the "partial derivative of e with respect to l"
  // I think this is right, but I don't know what was wrong with my reasoning.
  //  d_e_a(0) = f * (-xd * stheta + -yd * ctheta);
  //  d_e_a(1) = f * (xd * ctheta + - yd * stheta);
  d_e_a(0) = f * (-yd );
  d_e_a(1) = f * (xd);
  return d_e_a;
}

double TriLadder::compute_dtheta_internal_da(column_vector A,
			 column_vector B,
			 column_vector C,
			 column_vector D,
			 double a,
			 double b,
			 double c,
			 double f,
			 double g) {

  double a_c_B = a*a + c*c - b*b;
  double a_g_F = a*a + g*g - f*f;
  
  // no good way to name these...
  double nw = 1/c - a_c_B/(2.0*a*a*c);
  double sw = sqrt(1.0 - (a_c_B*a_c_B)/(4.0*a*a*c*c));
  double ne = 1/g - a_g_F/(2.0*a*a*g);
  double se = sqrt(1.0 - (a_g_F*a_g_F)/(4.0*a*a*g*g));

  double dchi_da = nw/sw; 
  double dbeta_da = ne/se;

  // Note this computes "does BC turn CW (<0) or CCW (>0) from AB,",
  // which is the opposite of "how does BA rotate into BC".
  double s_angle_abc = -sense(A(0),A(1),B(0),B(1),C(0),C(1));
  double s_angle_bcd = -sense(B(0),B(1),C(0),C(1),D(0),D(1));

  int sense_angle_abc_beta = int_sign(s_angle_abc);
  int sense_angle_bcd_chi = int_sign(s_angle_bcd);

  dbeta_da = dbeta_da * sense_angle_abc_beta;
  dchi_da = dchi_da * sense_angle_bcd_chi; 
  
  double dtheta_da = dchi_da + dbeta_da;
  
  return dtheta_da;
}


// The edge number is the full edge number, not numbered from the variable edges.
column_vector TriLadder::compute_internal_effector_derivative_c(column_vector cur_coords[],
								int edge_number,
								int goal_node_number) {
  // This is a preliminary attempt to code the internal derivative from "On Simple Planar Variable Geometry Trusses."
  // Given it's complexity, I suspect it will take some debugging.
  // This cannot be understood without reference to the diagram occuring in that paper.
  column_vector en = cur_coords[goal_node_number];	   
  int e = edge_number;
  int ai = small_node(e-2);
  int bi = large_node(e-2);
  int ci = bi+1;
  int di = ci+1;
  // this is a special case we are not prepared to handle yet!

  // WARNING: This is probably wrong -- we can't quite treat this as a special case
  // if there are multiple
  column_vector d_e_a(2);

  if (goal_node_number <= bi) {
    d_e_a(0) = 0.0;
    d_e_a(1) = 0.0;
    return d_e_a;
  } else if (goal_node_number == ci) {
    int last_node = ci;
    column_vector pen = cur_coords[last_node-1];
    double len = distance_2d(en,pen);
    d_e_a = (en - pen) / len;
    return d_e_a;
  } 

  
  column_vector A = cur_coords[ai];
  column_vector B = cur_coords[bi];
  column_vector C = cur_coords[ci];
  column_vector D = cur_coords[di];

  double a = distance_2d(B,C);
  double b = distance_2d(A,C);
  double c = distance_2d(A,B);
  double f = distance_2d(B,D);
  double g = distance_2d(C,D);

  //  column_vector en = cur_coords[goal_nodes[0]];

  column_vector X(2);
  X(0) = 100.0;
  X(1) = A(1);

  // Since these are signed, we actually use addition here...

  // I believe these need to be carefully calculated as signed angles...
  double rho = get_angle(B,A,X);
  double alpha = get_angle(C,A,B);
  // The sign of alpha is inportant as to waht teh rotation about C will be.

  column_vector rot(2);
  double angle_XAB = rho + alpha;
  // I am uncertain of this, I need to work it out more generally.
  if (alpha > 0) angle_XAB = -angle_XAB;
  
  rot(0) = sin(angle_XAB);
  rot(1) = -cos(angle_XAB);

  column_vector e_minus_c(2);
  e_minus_c(0) = -(en(1) - C(1));
  e_minus_c(1) = (en(0) - C(0));

  // using capital to represent negated terms...
  double b_c_A = b*b + c*c - a*a;
  double first_coeff = a / (c*sqrt( 1 - ((b_c_A*b_c_A) / (4*b*b*c*c))));
  double dtheta_da = compute_dtheta_internal_da(A,B,C,D,a,b,c,f,g);
  
  d_e_a = first_coeff*rot + dtheta_da*e_minus_c;
  return d_e_a;
}

void TriLadder::set_fixed_coords(column_vector coords[]) {
    column_vector temp0(2);
    temp0 = 0, 0;
    coords[0] = temp0;

    column_vector temp1(2);
    temp1 = 1.3, 0.75;
    coords[1] = temp1;
}


double FindCoords::operator() ( const column_vector& x) const
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

// Note sense computes "does BC turn CW or CCW from AB?"
// It does NOT compute the rotation of BA into BC.
double sense(double xa,double ya,double xb,double yb,double xc,double yc) {
  return (xb*yc + xa*yb + ya*xc) - (ya*xb + yb*xc + xa*yc);
}

int int_sign(double x) {
  if (x == 0.0)
    return 0;
  else
    return (x < 0.0) ? -1 : 1;
}

// This is an attempt to use FindCoords in the correct order to find all of the things

// bool debug_find_third = false;
// This this so that it doesn't modify *an but rather returns the coordinate vector
column_vector find_third_point_given_two_and_distances(Chirality sense,double dab,double dbc,double dca,
					      column_vector pa,column_vector pb) {
  if (debug_find_third) cout << "--------------\n";  
  if (debug_find_third) cout << "dab" << dab << "\n";
  if (debug_find_third) cout << "dbc" << dbc << "\n";
  if (debug_find_third) cout << "dca" << dca << "\n";
  if (debug_find_third) print_vec(pa);
  if (debug_find_third) print_vec(pb);

  
  // Conceptually, we create a straight line of length dca along x-axis...
  double x = dca;
  double y = 0.0;
  // Then we rotate it...we need to know rotaion of A->B vector and then andgle A
  double delta_x = pb(0) - pa(0);
  double delta_y = pb(1) - pa(1);
  if (debug_find_third) cout << "delta_x " << delta_x << "\n";
  if (debug_find_third) cout << "delta_y " << delta_y << "\n";  
  double theta = atan2(delta_y,delta_x); // radians?
  if (debug_find_third) cout << "theta " << theta*180/M_PI << "\n";
  if (debug_find_third) cout << "dca " << (dca*dca + dab*dab - dbc*dbc)/(2*dca*dab) << "\n";
  double v = (dca*dca + dab*dab - dbc*dbc)/(2*dca*dab);


  double A = (v >= 1.0) ? 0.0 : acos((dca*dca + dab*dab - dbc*dbc)/(2*dca*dab));
  
  if (debug_find_third) cout << "A " << A*180/M_PI << "\n";

  // I don't know how to compute what the sign of this operation should be.
  
  double diff = theta + ((sense == CCW) ? A : -A);
  
  if (debug_find_third) cout << "diff" << diff*180/M_PI << "\n";

  // we want to rotate by diff...
  y = sin(diff)*dca;
  x = cos(diff)*dca;
  
  // Then we translate by A into position...
  x += pa(0);
  y += pa(1);
  column_vector c(2);
  c = x,y;
  return c;
}

// const double SMALL_DEFINITION = 1.0e-3;
bool any_too_small(double a, double b, double c) {
  return ((a < SMALL_DEFINITION) || (b < SMALL_DEFINITION) || (c < SMALL_DEFINITION));
}

void fix_sense(bool desired_sense,double x0,double y0,double x1,double y1,column_vector z) {
  
      double det = sense(x0, y0, x1, y1, z(0), z(1));
      // d > 0 => ccw, d < 0 => cw.
      // now how do we make this penalize for going the wrong way?
      bool found_sense = (det > 0) ? CCW : CW;

      //      if (debug_find) std::cout << "det " << det << std::endl;
      
      bool need_to_flip = (desired_sense != found_sense);

      if (need_to_flip) {
	if (debug_find) std::cout << "NEED TO FLIP" << std::endl;
	if (debug_find) std::cout << "start: " << z << std::endl;
	column_vector r(2);
      
	// first we obtain the y = mx + b form..
	if ( (x0-x1) == 0) {
	  if (debug_find) std::cout << " SLOPE ZERO "  << std::endl;	  
	  // In this case, we have a vertical line, we can tell based on
	  // just where it is based on the x value of x and whether a is above
	  r = 2*x0 - z(0),z(1);
	} else {
	  const double m = (y0-y1) / (x0-x1);
	  const double b = y0 - m * x0;
	  const double d = (z(0) + (z(1) - b)*m)/(1+m*m);
	  const double xp = 2*d - z(0);
	  const double yp = 2*d*m - z(1) + 2 * b;
	  r = xp, yp;
	}
	if (debug_find) std::cout << "flipped: " <<  r  << std::endl;
	z = r;
      } else { // nothing to do
      }
}


void find_all_coords(TriLadder *an,column_vector coords[]) {
    FindCoords f;

    an->set_fixed_coords(coords);
    
    // Basic structure: Iteratively find coordinates based on simple triangulations.
    // This only works for actuator networks in which we can
    int fs = an->fixed_nodes.size();
    for(int i = fs; i < an->num_nodes; i++) {
      // Let's set up initial values
      // choose a starting point

      f.a.set_size(2);
      f.a = coords[i-fs];

      f.b.set_size(2);
      f.b = coords[i-(fs-1)];

      f.chi = ((i % 2) == 0) ? CCW : CW;

      // and minimize the function
      if (debug_find) std::cout << "f.a " << f.a << std::endl;
      if (debug_find) std::cout << "f.b " << f.b << std::endl;

      int e = an->edge_between(i,i-1);
      f.dac = an->distance(e-1);
      f.dbc = an->distance(e);
      
      double dab = distance_2d(f.a,f.b);
      column_vector  y(2);

      y = find_third_point_given_two_and_distances(f.chi,dab,f.dbc,f.dac,f.a,f.b);
      
      fix_sense(f.chi,f.a(0), f.a(1), f.b(0), f.b(1),y);


      if (any_too_small(dab,f.dbc,f.dac)) {
	cout << "TOO SMALL! \n";	
      } 

      coords[i].set_size(2);
      coords[i] = y;
      if (debug_find) std::cout << "f(y) " << f(y) << std::endl;
    }
    for (int i = 0; i < an->num_nodes; i++) {
      if (debug_find) std::cout << i << " =  " << an->coords[i] << std::endl;      
    }
};

