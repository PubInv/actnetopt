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


void physical_to_viewport(double px,double py,double *vx, double *vy);
void viewport_to_physical(double px,double py,double *vx, double *vy);

// const bool debug_find = false;
// const bool debug = false;

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

//class TriLadder {
//public:

  // // I know this is poor C++, I am not a very good C++ coder
  // //  typedef adjacency_list<vecS, vecS, bidirectionalS> Graph;
  // // Make convenient labels for the vertices
  // enum { A, B, C, D, E, F };
  // // Making this a const seems to destry to the implicit
  // // copy assignment; I have no idea why
  // static const int num_nodes = LADDER_NODES;
  // static const int num_edges = (num_nodes-3)*2 + 3;
  // static const int var_edges = num_edges-1;
  
  // const char* name = "ABCDE";

  // // writing out the edges in the graph
  // //  typedef std::pair<int, int> Edge;

  // //  Graph g;
  // //  Edge e;

  // //  static const int num_edges = 3;  
  // //  Edge edge_array[num_edges];
  // column_vector fixed_nodes;
  // int node_fixing_order[num_nodes];
  // //  typedef property_map<Graph, vertex_index_t>::type IndexMap;
  // //  typedef graph_traits<Graph>::vertex_iterator vertex_iter;
  // //  IndexMap index;
  // column_vector coords[num_nodes];
  // column_vector goals[1];

  // column_vector distance;
  
  // column_vector lower_bound;
  // column_vector upper_bound;
  
  // //  Eigen::VectorXd lower_bound;
  // //  Eigen::VectorXd upper_bound;  

  // // This is a map into the goal position for each goal.
  // // goal_nodes[a] = b => goals[a] should be considered node b.
  // std::vector<int> goal_nodes;

  TriLadder::TriLadder() {

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

// The goal of this is to compute a single derivative vector for the
// change in length to edge number edge_number.
// x and y are room for us to put in values.
// We compute the vector of positional change based on an actuator length
// change because it is inherently interesting; however, for minimization
// we in fact need to compute from this the change of the objective function.
// However, we hope that will be easy. We could instead compute the
// change in the objective function directly.
void TriLadder::compute_single_derivative(column_vector cur_coords[],
					     int edge_number,
					     double *x,
					     double *y) {
  *x = 3;
  *y = 4;
  
}
// I think this is the form actually required by functions such as find_min_box_constrained
// Probably this will have to call compute_single_derivative for each edge.
column_vector TriLadder::derivative (const column_vector& m) {
    // make us a column vector of length 2
    column_vector res(var_edges);
    for (int i = 0; i < var_edges; ++i) {
      double x;
      double y;
      compute_single_derivative(coords,i,&x,&y);
      double ds = compute_d_score_from_vector(x,y);
      res(i) = ds;
    }
    return res;
}

// };

//typedef enum { CW, CCW } Chirality;

// class FindCoords {
// public:
//   column_vector a;
//   column_vector b;    

//   TriLadder *an;
  
//   // Should the third point cc or ccw from a to b?
//   // In other words, we use this to disambiguate the two distance based solutions.
//   Chirality chi;

//   //     const double dab = 1.5; // This is in fact a constant in our frame
//   double dac; // these are in fact inputs to the problem
//   double dbc;
  
//   // this is just the objective (NOT optional)
//   // This input is an x,y position for c
//   // The lengths between a, b, and c are constants (effectively, input to the problem)
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
      double dab = distance(f.a,f.b);


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
    double dab = distance(a,b);
    double dbc = distance(b,c);
    double dca = distance(c,a);
    
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
  cout << "intermediate " << dot_product << "\n";
  
  double distance_product = distance(mp,bp)*distance(mp,bpp);
  cout << "d " << distance_product << "\n";

  bool prod_equal = (dot_product >= distance_product);
  
  cout << "prod_equal " << prod_equal << "\n";
  
  double theta = (dot_product >= distance_product) ? 0.0 : acos(dot(bpp-mp,bp-mp)/(distance(mp,bp)*distance(mp,bpp)));
  
  cout << "theta :" << theta*180/PI << "\n";
  
  column_vector cp(2);
  cp = rotate_point<double>(mp,tc,theta);
  column_vector dc(2);
  dc = cp - c;
  print_vec(tc); 
  print_vec(dc);
  cout << "====\n";
  return dc;
}
