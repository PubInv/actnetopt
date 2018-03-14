// Tetrahelix.cpp -- An optimizable tetrahelix manipulator
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
#include "Tetrahelix.hpp"

using namespace std;
using namespace dlib;

// Note edge numbering is really important, and I may be doing it
// differently here than in the javscript code.
// Basically I number edges in order of the lowest node, and then
// each of the (up to) 3 higher nodes. So edges are ordered by
// "lowest connected node, highest connected node".
// However, edges # 5 adn #6 are anomaolous, as they must complete
// the existing tetrahedron before falling into the standard pattern
// inorder to have a statically deteminant structure.

// Edge Num : sm node : lg node
// 0 : 0 : 1
// 1 : 0 : 2
// 2 : 0 : 3
// 3 : 1 : 2
// 4 : 1 : 3
// 5 : 2 : 3
// 6 : 1 : 4
// 7 : 2 : 4
// 8 : 3 : 4
// 9 :  2 : 5
// 10 : 3 : 5
// 11 : 4 : 5
int Tetrahelix::edges_in_tetrahelix(int n) {
  if (n >= 3) {
    return (n -2) *3;
  } else if (n == 2) {
    return 1;
  } else if (n == 1) {
    return 0;
  }
  return -1;
}

int Tetrahelix::large_node(int e) {
  if (e == 0) {
    return 1;    
  }
  if (e == 1) {
    return 2;        
  }
  if (e == 2) {
    return 3;        
  }
  if (e == 3) {
    return 2;        
  }
  if (e == 4) {
    return 3;        
  }
  if (e == 5) {
    return 3;
  }
  return (e/3) + 2;
}

int Tetrahelix::small_node(int e) {
  if (e == 0) {
    return 0;    
  }
  if (e == 1) {
    return 0;        
  }
  if (e == 2) {
    return 0;        
  }
  if (e == 3) {
    return 1;        
  }
  if (e == 4) {
    return 1;        
  }
  if (e == 5) {
    return 2;
  }
  return large_node(e) - (3 - (e % 3));
}

int edge_between_aux(int x,int y) {
  int d = y - x;
  if (y  == 0) {
    abort();
  }
  if (y == 1) {
    if (x == 0) {
      return 0;    
    }
  }
  if (y == 2) {
    if (x == 0) {
      return 1;        
    }
    if (x == 1) {
      return 3;        
    }
  }
  if (y == 3) {
    if (x == 0) {
      return 2;        
    }
    if (x == 1) {
      return 4;        
    }
    if (x == 2) {
      return 5;        
    }
  }
  int v = (y - 2)*3 + (3 - d);
  return v;
}

int Tetrahelix::edge_between(int x,int y) {
  if (abs(x-y) > 3) {
    abort();
    return -1;
  } else if (x == y) {
    abort();    
    return -1;
  } else return (x < y) ? edge_between_aux(x,y) : edge_between_aux(y,x);
}

Tetrahelix::Tetrahelix(int nodes,
		     double u,
		     double l,
		     double m,
		     double i)
{
  upper_bound_d = u;
  lower_bound_d = l;
  median_d = m;
  initial_d = i;

    num_nodes = nodes;
    
    num_edges = edges_in_tetrahelix(num_nodes);
    
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


void Tetrahelix::set_fixed_coords(column_vector coords[]) {
    column_vector temp0(3);
    temp0 = 0, 0, 0;
    coords[0] = temp0;

    column_vector temp1(3);
    temp1 = 0.0, 1.3, 0.75;
    coords[1] = temp1;
    
    column_vector temp2(3);
    temp1 = 0.75, 0.75, 1.5;
    coords[1] = temp1;
}

// In all probability, this can be unified with the 2D case
// by using the proper norming operations...
double FindCoords3d::operator() ( const column_vector& x) const
  {
    column_vector y(3);
    y  = x(0), x(1), x(2);

    
    column_vector ac(3);
    ac = y - a;

    column_vector bc(3);
    bc = y - b;

    column_vector cc(3);
    cc = y - c;
    

    double an = ac(0)*ac(0)+ac(1)*ac(1)+ac(2)*ac(2);
    double bn = bc(0)*bc(0)+bc(1)*bc(1)+bc(2)*bc(2);
    double cn = cc(0)*cc(0)+cc(1)*cc(1)+cc(2)*bc(2);
    return
      pow(sqrt(an) - dac,2) +
      pow(sqrt(bn) - dbc,2) +
      pow(sqrt(cn) - dcc,2);
  }
const int debug_find = 1;

// This is a tricky but essential routine. Given a triangle abc and three distances
// to a point d (da, db, dc), we have to find the point d.
// The best way to do this is to translate and rotate the point to
// the origin in a specific way, then comupte things simply, then perform the inverse transport.
// We are using the right-hand rule. Following computer graphichs convention, Y is condidered "up"
// and the Z dimenstion is considered depth.
// A is at the origin
// B is on the x axis (positive)
// C is in the x-y plane
// D is in the positive Z semiplane
// I need to work out the naming very clearly! That is a task for tomorrow.
// The input is the 6 distances.
// This code inspired by Dave Barber: view-source:http://tamivox.org/redbear/tetra_calc/index.html
column_vector find_point_from_transformed(double AB, double AC, double AD, double BC, double BD, double CD) {
  // _m2 means "squared"
  double AB_m2 = AB * AB; double AC_m2 = AC * AC;
     double AD_m2 = AD * AD; double BC_m2 = BC * BC;
     double BD_m2 = BD * BD; double CD_m2 = CD * CD;
     double qx = AB;
     cout << "AB " << AB << "\n";
     double rx = (AB_m2 + AC_m2 - BC_m2) / (2.0 * AB);    
     double ry = sqrt (AC_m2 - rx * rx);
     double sx = (AB_m2 + AD_m2 - BD_m2) / (2.0 * AB);
     double sy = (BD_m2 - (sx - qx) * (sx - qx) - CD_m2 + (sx - rx) * (sx - rx) + ry * ry) / (2 * ry);
     double sz = sqrt (AD_m2 - sx * sx - sy * sy);

     column_vector A(3);
     column_vector B(3);
     column_vector C(3);     
     A = 0.0,0.0,0.0;
     B = qx,0.0,0.0;
     cout << "BBB\n";
     print_vec(B);
     C = rx,ry,0.0;     
     //     A = new cart (0.0, 0.0, 0.0);
     //     B = new cart (qx,  0.0, 0.0);
     //     C = new cart (rx,  ry,  0.0);
     //     D = new cart (sx,  sy,  sz );
  column_vector D(3);
  D = sx,sy,sz;

  cout << "A B C D\n";
  print_vec(A);
  print_vec(B);
  print_vec(C);
  print_vec(D);
  return D;
}

// my attempt to compute the necessary transform
point_transform_affine3d compute_transform_to_axes(column_vector A, column_vector B, column_vector C) {
  // now we want to rotate the vector AB until it is pointing along the X axis.
  column_vector AB = B - A;

  // Crumb, column_vector doesn't seem to be a vector, which doesn't let me a normalize...
  dlib::vector<double,3> ABv(AB(0),AB(1),AB(2));
  dlib::vector<double,3> ABu = ABv.normalize();
  dlib::vector<double,3> Xu(1.0,0.0,0.0);
  cout << "Abu\n";

  cout << ABu;
  
  // Now, following: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
  dlib::vector<double,3> v = Xu.cross(ABu);
  cout << v;
  double s = v.length();
  double c = ABu.dot(Xu);

  cout << "s = " << s << "\n";
  cout << "c = " << c << "\n";
  // There is a possibility that c == 1.0 which must be handled...
  double q = 1/(1.0 +c);
  // Now we want a skew-symmetric cross-product matrix of v, according to the instructions...
  cout << "q = " << q << "\n";
  //  point_transform_affine3d I;
  //  point_transform_affine3d vx;
  dlib::matrix<double,3,3> vx;
  vx(0,0) = 0;
  vx(1,0) = v(3);
  vx(2,0) = -v(2);   

  vx(0,1) = -v(3);
  vx(1,1) = 0;
  vx(2,1) = v(1);   

  vx(0,2) = v(2);
  vx(1,2) = -v(1);
  vx(2,2) = 0;   

  cout << vx << "\n";
  dlib::matrix<double,3,3> vx2 = vx * vx;

  dlib::matrix<double,3,3> I;
  vx(0,0) = 1;
  vx(1,0) = 0;
  vx(2,0) = 0;   

  vx(0,1) = 0;
  vx(1,1) = 1;
  vx(2,1) = 0;   

  vx(0,2) = 0;
  vx(1,2) = 0;
  vx(2,2) = 1;   
  
  dlib::matrix<double,3,3> R = I + vx + vx2 * q;
  cout << R;
  //  dlib::vector<double,3> negA(-A(0),-A(1),-A(2));
  dlib::vector<double,3> zero(0,0,0);  
  point_transform_affine3d firstRotation(R,zero);
  point_transform_affine3d tform;
  translate_point(-A(0),-A(1),-A(2));  
  
  return firstRotation * translate_point(-A(0),-A(1),-A(2));
}
  
column_vector find_fourth_point_given_three_points_and_three_distances(Chirality sense,
								      column_vector pa,column_vector pb,column_vector pc,
								      double ad,double bd,double cd
								      ) {
  // First compute all 6 distances....
  double ab = distance_2d(pa,pb);
  double ac = distance_2d(pa,pc);
  double bc = distance_2d(pb,pc);

  // Now find transformation that rotates and translates to axes...
  

  // pa must move to the origin....

  // pb must move to the positive X axis....

  // pc must rotate into the x-z plane....
  // I think I will have to write my own routine for this...

  // Now get the fourth point...
  column_vector D = find_point_from_transformed(ab,ac,ad,bc,bd,cd);
  
  // Use inverse transform on the fourth point...
  return D;
}

void solve_forward_find_coords(Tetrahelix *an,column_vector coords[]) {
    FindCoords3d f;

    an->set_fixed_coords(coords);
    
    // Basic structure: Iteratively find coordinates based on simple triangulations.
    // This only works for actuator networks in which we can
    int fs = an->fixed_nodes.size();
    for(int i = fs; i < an->num_nodes; i++) {
      // Let's set up initial values
      // choose a starting point

      f.a.set_size(3);
      f.a = coords[i-fs];

      f.b.set_size(3);
      f.b = coords[i-(fs-1)];

      f.c.set_size(3);
      f.c = coords[i-(fs-2)];
      
      f.chi = ((i % 2) == 0) ? CCW : CW;

      // and minimize the function
      if (debug_find) std::cout << "f.a " << f.a << std::endl;
      if (debug_find) std::cout << "f.b " << f.b << std::endl;
      if (debug_find) std::cout << "f.c " << f.c << std::endl;      

      int e = an->edge_between(i,i-1);
      // his this right?
      f.dac = an->distance(e-2);
      f.dbc = an->distance(e-1);
      f.dcc = an->distance(e);

      column_vector  y(3);

      //      y = find_third_point_given_two_and_distances(f.chi,dab,f.dbc,f.dac,f.a,f.b);
      y = find_fourth_point_given_three_points_and_three_distances(f.chi,f.a,f.b,f.c,
								   f.dcc,f.dbc,f.dac);
      
      //      fix_sense(f.chi,f.a(0), f.a(1), f.b(0), f.b(1),y);


      //      if (any_too_small(dab,f.dbc,f.dac)) {
      //	cout << "TOO SMALL! \n";	
      //      } 

      coords[i].set_size(2);
      coords[i] = y;
      if (debug_find) std::cout << "f(y) " << f(y) << std::endl;
    }
    for (int i = 0; i < an->num_nodes; i++) {
      if (debug_find) std::cout << i << " =  " << an->coords[i] << std::endl;      
    }
};
