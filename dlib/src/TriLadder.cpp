//Using SDL and standard IO
// #include <SDL.h>
// #include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include <dlib/geometry.h>

#include <math.h>

#define PI 3.14159265

#include "TriLadder.h"

using namespace std;
using namespace dlib;


// ----------------------------------------------------------------------------------------

// In dlib, the general purpose solvers optimize functions that take a column
// vector as input and return a double.  So here we make a typedef for a
// variable length column vector of doubles.  This is the type we will use to
// represent the input to our objective functions which we will be minimizing.
// typedef matrix<double,0,1> column_vector;


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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void physical_to_viewport(double px,double py,double *vx, double *vy);
void viewport_to_physical(double px,double py,double *vx, double *vy);

// const bool debug_find = false;
// const bool debug = false;

double distance_2d(column_vector a, column_vector b) {
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


int TriLadder::edges_in_ladder(int n) {
  return (n-3)*2 + 3;
}

int TriLadder::large_node(int e) {
  return ((e % 2) == 0) ? (e/2)+1 : ((e+1)/2)+1 ;
}

int TriLadder::small_node(int e) {
  int L = large_node(e);
  return ((e % 2) == 1) ? L - 2 : L - 1;
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
      distance(i) = median_d;
    }
    
    for (int i = 0; i < var_edges; ++i) {
      lower_bound(i) = lower_bound_d;
      upper_bound(i) = upper_bound_d;            
    }
    int last_node = num_nodes - 1;
    double bx = ((last_node % 2) == 0) ? 0.0 : median_d * cos(30.0*PI/180);
    double by = (median_d/2.0) * last_node;
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
      //      std::cout << "Invert x " <<  x(0) <<  "," << x(1)  << std::endl;
      //      std::cout << "Invert y " <<  y(0) <<  "," << y(1)  << std::endl;                  
      d = x - y;
      if (debug) std::cout << "Invert d " <<  d(0) <<  "," << d(1) <<  " " << std::endl;
      double vp = d(0)*d(0) + d(1)*d(1);
      v += vp;
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
// I think this is computing the clockwise angle.
double get_angle(column_vector a, column_vector b, column_vector c) {
  column_vector Va = (a-b);
  column_vector Vc = (c-b);
  Va = normalize(Va);
  Vc = normalize(Vc);
  double aangle = atan2(Va(1), Va(0));
  double cangle = atan2(Vc(1), Vc(0));
  //  cout << "VA" << aangle*180/PI << "\n";
  //  cout << "VC" << cangle*180/PI << "\n";
  
  double angle = aangle - cangle;
  //  cout << angle*180/PI << "\n";
  if (angle >= PI) angle = -(2*PI - angle);
  if (angle <= -PI) angle = 2*PI + angle;

  //  cout << "phi" << angle*180/PI << "\n";  
  return angle;
}
// The goal of this is to compute a single derivative vector for the
// change in length to edge number edge_number.
// x and y are room for us to put in values.
// We compute the vector of positional change based on an actuator length
// change because it is inherently interesting; however, for minimization
// we in fact need to compute from this the change of the objective function.
// However, we hope that will be easy. We could instead compute the
// change in the objective function directly.
double TriLadder::compute_single_derivative_dtheta(column_vector cur_coords[],
					     int edge_number) {
  
  // We add one because the variable edges are not the same as the num_edges
  int e = edge_number + 1;
  cout << "edge  "<< e << "\n";  
  // Case split on if the edge is an outside edge or not
  // A B C are a triangle.  A and B are fixed; BC is the changing line
  // (the one specified by edge_number.) phi is angle ABC.

  int C = large_node(e);
  //  cout << "C " << C << "\n";    
  column_vector c = cur_coords[C];    
  int B = small_node(e);
  //   cout << "B " << B << "\n";  
  column_vector b = cur_coords[B];

  int A;
  if ((C - 1) == B) {
    A = C - 2;
  } else { 
    A = C - 1;
  }
  
  //  cout << "A " << A << "\n";
  column_vector a = cur_coords[A];

  double len = distance_2d(a,c);
  
  double phi = get_angle(a,b,c);

  // cout << A << " " << B << " " << C << " a,b,c\n";
  // print_vec(a);
  // print_vec(b);
  // print_vec(c);
  cout << phi*180/PI << "\n";
  double dThetaDx = 1.0 / (len*sin(phi));
  
  return dThetaDx;
  
}
column_vector TriLadder::compute_single_derivative_c(column_vector cur_coords[],
						  int edge_number) {
  int e = edge_number + 1;
  int C = large_node(e);
  int B = small_node(e);
  int A;
  if ((C - 1) == B) {
    A = C - 2;
  } else { 
    A = C - 1;
  }
  double dtheta = compute_single_derivative_dtheta(cur_coords,edge_number);

  // Now that we have dtheta, we want to compute the dCx/dtheta and dCy/dtheta
  // dtheta is the rotation of C into A
  // dCx,dCy are perpendicular to AC and proportional to dtheta and
  // the sign depends where C, B, and A are.
  // one way to do this is to rotate C into A about B and see how the
  // values change to deterimne the sign.

  column_vector c = cur_coords[C];    
  column_vector b = cur_coords[B];
  column_vector a = cur_coords[A];
  column_vector bc = c - b;
  double len = distance_2d(b,c);
  bc = normalize(bc);
  double phi = atan2(bc(1), bc(0));
  double alpha = PI/2.0 - phi;
  double dy = cos(alpha)*len;
  double dx = sin(alpha)*len;
  column_vector d(2);
  // do we just multiply by dtheta?  
  d(0) = dx*dtheta;
  d(1) = dy*dtheta;
  return d;
}

column_vector TriLadder::compute_external_effector_derivative_c(column_vector cur_coords[],
						  int edge_number) {
  // First, let us set up our variables...
  // This nomenclature matches the paper "OnSimplePlanarVariableGeomertyTrusses"
  // e is the end effector
  column_vector en = cur_coords[goal_nodes[0]];
  double enx = en(0);
  double eny = en(1);

  // let P be the joint about which we are rotating by changing e
  // let M be the directly moved joint.
  // let S be the "stable" joint connected to M by e.

  int e = edge_number+1;
  int M = large_node(e);
  int S = small_node(e);
  int P = (M+S)/2;
  // note these nodes should be three-in-a-row
  if (debug) cout << "S,P,M:" << S << " " << P << " " << M << "\n";

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

  if (debug) cout << "f = " << f << "\n";

  // theta is angle about P of S into M
  // I worked the math in that paper out based on anticlockwise angles!
  //  double theta = -get_angle(s,p,m);
  column_vector xaxis(2);
  xaxis(0) = 10.0;
  xaxis(1) = 0.0;

  column_vector pivot_to_en = en - p;
  //  cout << "en :" << en << "\n";    
  //  cout << "pivot_to_en :" << pivot_to_en << "\n";
  //  cout << "p :" << p << "\n";  
  double theta = -get_angle(xaxis+p,p,en);
  
  double stheta = sin(theta);
  double ctheta = cos(theta);

  if (debug) cout << "theta :" << theta*180/PI << "\n";
  if (debug) cout << "stheta :" << stheta << "\n";
  if (debug) cout << "ctheta :" << ctheta << "\n";

  double xd = (enx - x);
  double yd = (eny - y);
  if (debug) cout << "xd, yd" << xd << "," << yd << "\n";

  // d_e_l is the "partial derivative of e with respect to l"
  column_vector d_e_l(2);
  // I think this is right, but I don't know what was wrong with my reasoning.
  //  d_e_l(0) = f * (-xd * stheta + -yd * ctheta);
  //  d_e_l(1) = f * (xd * ctheta + - yd * stheta);
  d_e_l(0) = f * (-yd );
  d_e_l(1) = f * (xd);
  return d_e_l;
}

column_vector TriLadder::compute_internal_effector_derivative_c(column_vector cur_coords[],
						  int edge_number) {
  // This is a preliminary attempt to code the internal derivative from "On Simple Planar Variable Geometry Trusses."
  // Given it's complexity, I suspect it will take some debugging.
  // This cannot be understood without reference to the diagram occuring in that paper.

  int e = edge_number+1;
  int ai = small_node(e);
  int bi = large_node(e);
  int ci = bi+1;
  int di = ci+1;

  cout << "e " << e << "\n";
  cout << "ai bi ci di " << ai << " " << bi << " " << ci << " " << di << "\n";

  column_vector A = cur_coords[ai];
  column_vector B = cur_coords[bi];
  column_vector C = cur_coords[ci];
  column_vector D = cur_coords[di];    
  
  double a = distance_2d(B,C);
  double b = distance_2d(A,C);
  double c = distance_2d(A,B);
  double f = distance_2d(B,D);
  double g = distance_2d(C,D);

  column_vector en = cur_coords[goal_nodes[0]];

  column_vector X(2);
  X(0) = 100.0;
  X(1) = 0.0;

  double rho = get_angle(B,A,X);
  double alpha = get_angle(C,A,B);

  //  cout << "rho :" << rho*180/M_PI << "\n";
  //   cout << "alpha :" << alpha*180/M_PI << "\n";      

  

  column_vector rot(2);
  rot(0) = sin(rho-alpha);
  rot(1) = -cos(rho-alpha);

  column_vector e_minus_c(2);
  e_minus_c(0) = -(en(1) - C(1));
  e_minus_c(1) = (en(0) - C(0));


  cout << "rot:" << "\n";
  print_vec(rot);
  
  cout << "e_minus_c:" << "\n";  
  print_vec(e_minus_c);
  

  // using capital to represent negated terms...
  double b_c_A = b*b + c*c - a*a;
  double a_c_B = a*a + c*c - b*b;
  double a_g_F = a*a + g*g - f*f;

  cout << "a,b,c,f,g " << a << " " << b << " " << c << " " << f << " " << g << "\n";

  cout << "b_c_A " << b_c_A << "\n";

  cout << "denom " << 4*b*b*c*c << "\n";  

  cout << "LARGER THAN 1.0? " << ((b_c_A*b_c_A) / (4*b*b*c*c)) << "\n";
  

  double first_coeff = a / (c*sqrt( 1 - ((b_c_A*b_c_A) / (4*b*b*c*c))));

  cout << "first_coeff: " << first_coeff << "\n";  

  
  // no good way to name these...
  double nw = 1/c - a_c_B/(2.0*a*a*c);
  double ne = 1/g - a_g_F/(2.0*a*a*g);  
  double sw = sqrt(1.0 - (a_c_B*a_c_B)/(4.0*a*a*c*c));
  double se = sqrt(1.0 - (a_g_F*a_g_F)/(4.0*a*a*g*g));

  double second_coeff = (nw/sw) - (ne/se);
  cout << "second_coeff: " << second_coeff << "\n";
  

  // derivative of end effector wrt to change in length a...
  column_vector d_e_a(2);
  d_e_a = first_coeff*rot + second_coeff*e_minus_c;
  return d_e_a;
}

// I think this is the form actually required by functions such as find_min_box_constrained
// Probably this will have to call compute_single_derivative for each edge.
column_vector TriLadder::derivative (const column_vector& m) {
    // make us a column vector of length 2
    column_vector res(var_edges);
    for (int i = 0; i < var_edges; ++i) {
      //      double x;
      //      double y;
      double dtheta = compute_single_derivative_dtheta(coords,i);

      //      double ds = compute_d_score_from_vector(x,y);
      //      res(i) = ds;
      res(i) = dtheta;
    }
    return res;
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
// };

// This is really a determinant of a 3x3 matrix with a column on the right
double sense(double a, double b, double d, double e, double g, double h) {
  return a*e - a*h - b*d + b*g + d*h - e*g;
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
  c(0) = x;
  c(1) = y;
  if (debug_find_third) print_vec(c);
  if (debug_find_third) cout << "======\n";
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

      if (debug_find) std::cout << "det " << det << std::endl;
      
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
	z(0) = r(0);
	z(1) = r(1);
      }
      if (debug_find) std::cout << " z " << z << std::endl;
}

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
      double dab = distance_2d(f.a,f.b);


      column_vector  y(2);

      y = find_third_point_given_two_and_distances(desired_sense,dab,f.dbc,f.dac,f.a,f.b);
      
      fix_sense(desired_sense,f.a(0), f.a(1), f.b(0), f.b(1),y);

      // column_vector  x(2);
      // x = 1, 2;
      // find_min_using_approximate_derivatives(bfgs_search_strategy(),
      // 					     objective_delta_stop_strategy(1e-7),
      // 					     f, x, -1);
      // fix_sense(desired_sense,f.a(0), f.a(1), f.b(0), f.b(1),x);      

      //      if (distance(x,y) > 0.01) {
	//	cout << "dist " << distance(x,y) << "\n";
	//	print_vec(x);
	//	print_vec(y);
      //      }
      
      column_vector  z(2);            
      if (!any_too_small(dab,f.dbc,f.dac)) {
	z = y;
      } else {
	cout << "TOO SMALL! \n";
	//	z = x;
      }
      z = y;

      //      fix_sense(desired_sense,f.a(0), f.a(1), f.b(0), f.b(1),z);            
      // double det = sense(f.a(0), f.a(1), f.b(0), f.b(1), z(0), z(1));
      // // d > 0 => ccw, d < 0 => cw.
      // // now how do we make this penalize for going the wrong way?
      // bool found_sense = (det > 0) ? CCW : CW;

      // if (debug_find) std::cout << "processing node: " << i << std::endl;
      
      // if (debug_find) std::cout << "det " << det << std::endl;
      
      // bool need_to_flip = (desired_sense != found_sense);

      // if (need_to_flip) {
      // 	if (debug_find) std::cout << "NEED TO FLIP" << std::endl;
      // 	if (debug_find) std::cout << "start: " << z << std::endl;
      // 	column_vector r(2);
      
      // 	// first we obtain the y = mx + b form..
      // 	if ( (f.a(0)-f.b(0)) == 0) {
      // 	  if (debug_find) std::cout << " SLOPE ZERO "  << std::endl;	  
      // 	  // In this case, we have a vertical line, we can tell based on
      // 	  // just where it is based on the x value of x and whether a is above
      // 	  r = 2*f.a(0) - z(0),z(1);
      // 	} else {
      // 	  const double m = (f.a(1)-f.b(1)) / (f.a(0)-f.b(0));
      // 	  const double b = f.a(1) - m * f.a(0);
      // 	  const double d = (z(0) + (z(1) - b)*m)/(1+m*m);
      // 	  const double xp = 2*d - z(0);
      // 	  const double yp = 2*d*m - z(1) + 2 * b;
      // 	  r = xp, yp;
      // 	}
      // 	if (debug_find) std::cout << "flipped: " <<  r  << std::endl;
      // 	z(0) = r(0);
      // 	z(1) = r(1);
      // }
      // if (debug_find) std::cout << " x " << x << std::endl;

      coords[i].set_size(2);
      coords[i](0) = z(0);
      coords[i](1) = z(1);
      
      // print argmin
      // Now take x and make it the first coord...
      
      if (debug_find) std::cout << "f(x) " << f(z) << std::endl;
    }
    for (int i = 0; i < an->num_nodes; i++) {
      if (debug_find) std::cout << i << " =  " << an->coords[i] << std::endl;      
    }
};



void   test_find_third_point_given_two_and_distances() {
    column_vector a(2);
    column_vector b(2);
    column_vector c(2);
    column_vector x(2);            
    a(0) = 0.0;
    a(1) = 0.0;
    b(0) = 1.3;
    b(1) = 0.75;
    c(0) = 2.0;
    c(1) = 0.0;
    double dab = distance_2d(a,b);
    double dbc = distance_2d(b,c);
    double dca = distance_2d(c,a);
    
    x = find_third_point_given_two_and_distances(CCW,dab,dbc,dca,a,b);
    print_vec(c);
    print_vec(x);
    cout << "XXXXXXXXXX\n";
}

column_vector change_in_third_point(column_vector a,
				    column_vector b,
				    column_vector c,
				    column_vector da,
				    column_vector db
				    ) {
  // This is the translation of the midpoint of a and b
  column_vector dm(2);
  dm = (da + db)/2;
  column_vector m(2);
  m = (a+b)/2;
  column_vector mp(2);
  mp = m+dm;

  // tc is c translated, but not rotated
  column_vector tc(2);
  tc = c + dm;
  
  column_vector dzr(2);
  column_vector bp(2);
  bp = b + dm;
  column_vector bpp(2);
  bpp = b + db;
  // now theta is the angle bpp->m->bp
  // Sadly as always we have to worry about the sign here...
  double dot_product = dot(bpp-mp,bp-mp);
  if (debug) cout << "intermediate " << dot_product << "\n";
  
  double distance_product = distance_2d(mp,bp)*distance_2d(mp,bpp);
  if (debug) cout << "d " << distance_product << "\n";

  bool prod_equal = (dot_product >= distance_product);
  
  if (debug) cout << "prod_equal " << prod_equal << "\n";
  
  double theta = (dot_product >= distance_product) ? 0.0 : acos(dot(bpp-mp,bp-mp)/(distance_2d(mp,bp)*distance_2d(mp,bpp)));
  
  if (debug) cout << "theta :" << theta*180/PI << "\n";
  
  column_vector cp(2);
  cp = rotate_point<double>(mp,tc,theta);
  column_vector dc(2);
  dc = cp - c;
  print_vec(tc); 
  print_vec(dc);
  cout << "====\n";
  return dc;
}
