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
//
// Note: This means that edges 0,1,3 are not variable, but fixed,
// which is rather confusing.
int Tetrahelix::edge_number_of_nth_variable_edge(int n) {
  if (n == 0) {
    return 2;
  } else if (n == 1) {
    return 4;
  } // else if (n >= 2) {
    return n + 3;
    //  }
}

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

// under the current numbering, externals are odd...
bool Tetrahelix::rail_edge(int edge) {
  // WARNING--this needs to be rewritten -- not sure how this works for tetrahelix...
  // really the questin is are we changing rails, which is best done where we can
  // detect the small and large node numbers.
  
  return ((edge % 2) == 1);
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

    // Note: This is wildly different than the 2d case, which requires just one fixed edge.
    var_edges = num_edges-3;
    node_fixing_order = new int[num_nodes];
    coords = new column_vector[num_nodes];      

    fixed_nodes.set_size(3);
    fixed_nodes(0) = A;
    fixed_nodes(1) = B;
    fixed_nodes(2) = C;    

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

    double f = 3;
    // These values are taken from: https://pubinv.github.io/tetrahelix/
    temp0 = 0.25980762113533157,  0.7794228634059948,  -1;
    coords[0] = temp0*f;


    temp0 =  -0.17320508075688773, 0.9730720307163656, -0.841886116991581;   
    coords[1] = temp0*f;
    
    temp0 =  -0.028867513459481214,  0.5212239736588337, -0.683772233983162;
    coords[2] = temp0*f;

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
const int debug_find = 0;

// This is a tricky but essential routine. Given a triangle abc and three distances
// to a point d (da, db, dc), we have to find the point d.
// The best way to do this is to translate and rotate the point to
// the origin in a specific way, then comupte things simply, then perform the inverse transport.
// We are using the right-hand rule. Following computer graphichs convention, is condidered "up"
// and the Z dimenstion is considered depth.
// A is at the origin
// B is on the x axis (positive)
// C is in the x-y plane
// D is in the positive Z semiplane
// I need to work out the naming very clearly! That is a task for tomorrow.
// The input is the 6 distances.
// This code inspired by Dave Barber: view-source:http://tamivox.org/redbear/tetra_calc/index.html
column_vector find_point_from_transformed(Chirality sense,double AB, double AC, double AD, double BC, double BD, double CD) {
  // _m2 means "squared"
  double AB_m2 = AB * AB; double AC_m2 = AC * AC;
     double AD_m2 = AD * AD; double BC_m2 = BC * BC;
     double BD_m2 = BD * BD; double CD_m2 = CD * CD;
     double qx = AB;
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
     column_vector D(3);
     D = sx,sy,(sense == CCW) ? sz : -sz;

  // cout << "A B C D\n";
  // print_vec(A);
  // print_vec(B);
  // print_vec(C);
  // print_vec(D);
  return D;
}

// my attempt to compute the necessary transform
// The purpose of this is to create the transform that makes A->B point along the X axis, I think.
// point_transform_affine3d compute_transform_to_axes(column_vector A, column_vector B, column_vector C) {
//   // now we want to rotate the vector AB until it is pointing along the X axis.
//   column_vector AB = B - A;

//   // Crumb, column_vector doesn't seem to be a vector, which doesn't let me a normalize...
//   dlib::vector<double,3> ABv(AB(0),AB(1),AB(2));
//   dlib::vector<double,3> ABu = ABv.normalize();
//   dlib::vector<double,3> Xu(1.0,0.0,0.0);
//   cout << "Abu\n";

//   cout << ABu;
  
//   // Now, following: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
//   dlib::vector<double,3> v = ABu.cross(Xu);
//   cout << v;
//   double s = v.length();
//   double c = ABu.dot(Xu);
 

//   cout << "s = " << s << "\n";
//   cout << "c = " << c << "\n";
//   // There is a possibility that c == 1.0 which must be handled...
//   double q = 1/(1.0 +c);
//   // Now we want a skew-symmetric cross-product matrix of v, according to the instructions...
//   cout << "q = " << q << "\n";
//   //  point_transform_affine3d I;
//   //  point_transform_affine3d vx;
//   dlib::matrix<double,3,3> vx;
//   vx(0,0) = 0;
//   vx(1,0) = v(3);
//   vx(2,0) = -v(2);   

//   vx(0,1) = -v(3);
//   vx(1,1) = 0;
//   vx(2,1) = v(1);   

//   vx(0,2) = v(2);
//   vx(1,2) = -v(1);
//   vx(2,2) = 0;   

//   dlib::matrix<double,3,3> vx2 = vx * vx;
//   cout << "vx* vx \n";
//   cout << vx2;

//   dlib::matrix<double,3,3> I;
//   I(0,0) = 1;
//   I(1,0) = 0;
//   I(2,0) = 0;   

//   I(0,1) = 0;
//   I(1,1) = 1;
//   I(2,1) = 0;   

//   I(0,2) = 0;
//   I(1,2) = 0;
//   I(2,2) = 1;   
  
//   dlib::matrix<double,3,3> R = I + vx + (vx2) ;
//   cout << "R = \n";
//   cout << R;
//   //  dlib::vector<double,3> negA(-A(0),-A(1),-A(2));

//   cout << "Attempt to rotate by matrix multiply\n";
//   cout << R*AB;
//   dlib::vector<double,3> zero(0,0,0);  
//   point_transform_affine3d firstRotation(R,zero);
//   point_transform_affine3d tform;
  
//   return firstRotation * translate_point(-A(0),-A(1),-A(2));
// }

// This is a slighlty different way based on answer by Kuba Ober in ths same way.
// This tries to rotate the vector A-B on to the X-Axis. C should end up in the XY plane.
// I think if the the vector A-B points in the negative x direction, we have a special case
// that causes a problem here. In that case we are 180 degrees out of sync. I need to think
// carefully about how to handle this.
point_transform_affine3d compute_transform_to_axes2(column_vector pA, column_vector pB, column_vector pC) {

  // first we translate to the origin
  point_transform_affine3d trans = translate_point(-pA(0),-pA(1),-pA(2));
  column_vector pAp = trans(pA);
  column_vector pBp = trans(pB);
  column_vector pCp = trans(pC);
  

  // Based on Kuba Ober's answer we need to compute F And G.
  // now we want to rotate the vector AB until it is pointing along the X axis.
  column_vector AB = pBp - pAp;

  // Crumb, column_vector doesn't seem to be a vector, which doesn't let me a normalize...
  dlib::vector<double,3> Avec(AB(0),AB(1),AB(2));
  dlib::vector<double,3> A = Avec.normalize();
  dlib::vector<double,3> B(1.0,0.0,0.0);
  // now both A and B are unit vectors.
  
  dlib::matrix<double,3,3> U;

  dlib::vector<double,3> AxB = A.cross(B);

  double s = AxB.length();
  double c = A.dot(B);

  point_transform_affine3d alignX;

  if (c == -1) {
      // This means that A and B point in opposite directions, and  must do something else
      // In this case the rotation can be Pi around either the z or y axis to get us
      // where we need to be...I prefer to ratate and z
      alignX =  rotate_around_z(M_PI)* trans;    
  } else {
    if (equal(A,B)) {
      U = 1,0,0,
	0,1,0,
	0,0,1;
    } else {
    // Now, following: https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
      //      cout << "s = " << s << "\n";
      //      cout << "c = " << c << "\n";
      // There is a possibility that c == 1.0 which must be handled...
      // Now we want a skew-symmetric cross-product matrix of v, according to the instructions...
      //  point_transform_affine3d I;
      //  point_transform_affine3d vx;
      dlib::matrix<double,3,3> G;
      G(0,0) = c;
      G(1,0) = s;
      G(2,0) = 0;   

      G(0,1) = -s;
      G(1,1) = c;
      G(2,1) = 0;   

      G(0,2) = 0;
      G(1,2) = 0;
      G(2,2) = 1;   

      dlib::vector<double,3> u = A;
      // NOTE: if A == B, this is a real problem. A special case that must be handled.
      // cout << "A = " << A << "\n";
      // cout << "B = " << B << "\n";        
      // cout << "B - c*A) = " << B - c*A << "\n";    
      dlib::vector<double,3> v = (B - c*A).normalize();
      //      cout << " v = " << v << "\n";

      dlib::vector<double,3> w = B.cross(A);

      dlib::matrix<double,3,3> F;
      F(0,0) = u(0);
      F(1,0) = u(1);
      F(2,0) = u(2);   

      F(0,1) = v(0);
      F(1,1) = v(1);
      F(2,1) = v(2);   

      F(0,2) = w(0);
      F(1,2) = w(1);
      F(2,2) = w(2);   
  
      dlib::matrix<double,3,3> Finv = inv(F);
      U = F * G * Finv;
    }
    dlib::vector<double,3> zero(0,0,0);  
    point_transform_affine3d firstRotation(U,zero);

    // now we have a transform that translates point B onto the X Axis.
    // We now desire to rotate the point C into the XY plane.
    // This can be handled more simply as a rotation about the X-Axis.
    // So we need to compute the angle to rotate, call it theta.
    // theta = asin(C_y)
    alignX =  firstRotation * trans;
  } 

  // Here we rotate C via our transform....
  column_vector Cn = alignX(pC);
  
   // cout << "Cn\n";
   // print_vec(Cn);

  //   double len = alignX(pC).length();
   // cout << "len " << len << "\n";
   // cout << "Cn(2) " << Cn(2) << "\n";
   // cout << Cn(0)*Cn(0) + Cn(1)*Cn(1) << "\n";
  
  double denom = sqrt(Cn(0)*Cn(0) + Cn(1)*Cn(1));
  // cout << "denom: " << denom << "\n";
  // cout << Cn(0)*Cn(0) + Cn(1)*Cn(1) << "\n";
  
  double value = Cn(2)/denom;

  //  cout << "value " << value << "\n";
  //  cout << "near zero " << abs(value - 1.0) << "\n";    
  double theta;
  if (abs(value - 1.0) < 1e-3) {
    //    cout << "UNITARY\n";
    theta = -M_PI/2;
  } else {
    // how do we know this is the correct sign?    
    theta = -atan2(Cn(2),Cn(1));
  }
  //  cout << "theta: " << theta*180/(M_PI) << "\n";
  // Now, if theta is a multiple of 180 degrees, we don't really need to rotate...
  if ((((int) round(theta*180/(M_PI))) % 180) == 0) {
    // cout << "Yes, this happened to be a 180 rotation, so avoiding!\n";
    // cout << "pc " << pC << "\n";
    // cout << alignX(pC) << "\n";
    // cout << alignX.get_m() << "\n";
    // cout << alignX.get_b() << "\n";  
    return rotate_around_x(0) *alignX;    
  } else {
    return rotate_around_x(theta) * alignX;
  }
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
  //  cout << "pa pb pc \n";
  //  cout << pa << " " << pb << " " << pc << "\n";  
  point_transform_affine3d tform = compute_transform_to_axes2(pa,pb,pc);

  column_vector Ap(3);
  column_vector Bp(3);
  column_vector Cp(3);

  Ap = tform(pa);
  // cout << "Ap \n";
  // print_vec(Ap);
  
  Bp = tform(pb);
  // cout << "Bp \n";
  // print_vec(Bp);
  
  Cp = tform(pc);
  // cout << "Cp \n";
  // print_vec(Cp);
  
  point_transform_affine3d tform_inv = inv(tform);
  // cout << "tform inv\n";
  // cout << tform_inv.get_m();

  //  column_vector X(3);
  //  X = 1.0,1.0,1.0;
  //  double phi = M_PI/6;
  // point_transform_affine3d rotate0 = rotate_around_x(phi);
  // cout << "XXXXXXX\n";
  // cout << X << "\n";
  // cout << rotate0(X) << "\n";
  // cout << rotate0(inv(rotate0)(X)) << "\n";
  // cout << "XXXXXXX\n";  

  

  // Now get the fourth point...
  //  cout << ab << " " << ac << " " << ad << " " << bc << " " << bd << " " << cd << "\n";
  column_vector D = find_point_from_transformed(sense,ab,ac,ad,bc,bd,cd);
  return tform_inv(D);
}

void solve_forward_find_coords(Tetrahelix *an,column_vector coords[]) {
    FindCoords3d f;

    Chirality sense = CCW;
    an->set_fixed_coords(coords);

    cout << "in solve distances:\n";
    for(int i = 0; i < an->num_edges; i++) {
      cout << "i, d " << i << " , " << an->distance(i) << "\n";
    }

    
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

      // Although in 2-dimensions we change this, in 3D it is not needed...
      // I think there is something deeply mathmatical in that.
      f.chi = sense;

      // and minimize the function
      if (debug_find) std::cout << "f.a " << f.a << std::endl;
      if (debug_find) std::cout << "f.b " << f.b << std::endl;
      if (debug_find) std::cout << "f.c " << f.c << std::endl;      

      int ec = an->edge_between(i,i-1);
      int eb = an->edge_between(i,i-2);
      int ea = an->edge_between(i,i-3);            

      f.dac = an->distance(ea);
      f.dbc = an->distance(eb);
      f.dcc = an->distance(ec);

      column_vector  y(3);

      // cout << "args : \n";
      // cout << f.chi << "\n";
      // print_vec(f.a);
      // print_vec(f.b);
      // cout << "f.c \n";
      // print_vec(f.c);
      cout << f.dcc << "\n";
      cout << f.dbc << "\n";
      cout << f.dac << "\n";            
      y = find_fourth_point_given_three_points_and_three_distances(f.chi,f.a,f.b,f.c,
								   f.dcc,f.dbc,f.dac);
      
      coords[i].set_size(3);
      coords[i] = y;
      //      cout << "SETTING " << i << "\n";
      //      print_vec(y);
      if (debug_find) std::cout << "f(y) " << f(y) << std::endl;
    }
    for (int i = 0; i < an->num_nodes; i++) {
      if (debug_find) std::cout << i << " =  " << an->coords[i] << std::endl;      
    }
};

void Tetrahelix::add_goal_node(int num,double x,double y, double z,double w)    {
      column_vector mg(3);
      mg(0) = x;
      mg(1) = y;
      mg(2) = z;
      goals.push_back(mg);
      goal_nodes.push_back(num);
      goal_weights.push_back(w);
      cout << "QQQ " << goals.size() << "\n";
    }


double dtheta_dd_laid_on_axes(double d, double x, double z) {
  //  cout << " d, x, z " <<  d << " " << x << " " << z << "\n";
  double lower_term = pow((-d*d + x*x + z*z + 1.0),2)/(4.0*x*x);
  //  cout << " lower " << lower_term << "\n";
  return d / (
	      x * sqrt(1.0 - lower_term));
}

// In this case, x,y,z are the lengths of a triangle
double darea_dx(double x,double y, double z) {
  double x2 = x*x;
  double y2 = y*y;
  double z2 = z*z;
  double num = x*(-x2 + y2 + z2);
  double dem = 2*sqrt(-x2*x2+ 2*x2*y2 + 2*x2*z2 - y2*y2 + 2*y2*z2 - z2*z2);
  return num/dem;
}

// a,b,c,d are the areas of the triangles opposite i,j,k,l respectively.
// e,f,g and dihedral angles of il,ik, and ij respectively.
// our basic goal is to compute dihedral angles; this is part of that.
double dangle_db(double a, double b, double c, double d, double e, double f) {
  double num = -(2*b - 2*c*cos(e) - 2*d*cos(f));
  double lnum = -a*a + b*b - 2*b*c*cos(e) - 2*b*d*cos(f)+c*c + d*d;
  double lden = 4*c*c * d*d;
  double den = 2*c*d * sqrt(1 - (lnum*lnum)/lden);
  return num/den;
}
double semi_perimeter(double x, double y, double z) {
  return (x+y+z)/2.0;
}
double heron_area(double x,double y, double z) {
  double s = semi_perimeter(x,y,z);
  return sqrt(s*(s-x)*(s-y)*(s-z));
}

// find the dihedral angle of <AB given vertex angles...
double dihedral_from_vertex_angles(double a, double b, double c) {
  double num = cos(c) - cos(b)*cos(a);
  double den = sin(b) * sin(a);
  return acos(num/den);
}

double angle_from_three_sides(double adjacent1, double adjacent2, double opposite) {
  double a = adjacent1;
  double b = adjacent2;
  double c = opposite;
  return acos((a*a + b*b - c*c)/(2*a*b));
}
double dangle_from_dside(double adjacent1, double adjacent2, double opposite) {
  double a = adjacent1;
  double b = adjacent2;
  double c = opposite;
  double num = c;
  double inum = pow(a*a + b*b - c*c,2);
  double iden = 4 * a * a * b * b;
  double den = a * b * sqrt(1 - inum/iden);
  return num/den;
}

double ddhedral_dvertex(double adjacent1, double adjacent2, double opposite) {
  double a = adjacent1;
  double c = adjacent2;
  double b = opposite;
  double num = csc(a)*csc(b)*sin(c);
  double den = sqrt(1 - csc(a)*csc(a) * csc(b)*csc(b) * pow( cos(c) - cos(a)*cos(b),2));
  return -num/den;
}

// Compute the ccw rotation by theta about the segment A->B of M (moving) by theta
// TODO: This needs to be decleared in the .hpp place.

// Note: The test of this is insufficient.
// Furthermore, we need to decide if the order of A, B matters (does it change the direction?)
// Yes, we need to think of this as an anti-clockwise rotation. If A and B are swapped,
// the rotation is in the other direction.
// Note: Under some test conditions, compute_transform_to_axes is failing and producing NANs.
// Need to put that under test.
column_vector compute_rotation_about_points(column_vector A,
					    column_vector B,
					    double theta,
					    column_vector M)
{

  // cout << "A B\n";
  // print_vec(A);
  // print_vec(B);
  
  point_transform_affine3d tform = compute_transform_to_axes2(A,B,M);
  // cout << tform.get_m() << "\n";
  // cout << tform.get_b() << "\n";  
  

  column_vector M_on_axis = tform(M);
  
  // cout << "M_on_axis \n";
  // cout << M_on_axis;
  

  point_transform_affine3d rotate0 = rotate_around_x(theta);
  column_vector M_rotated = rotate0(M_on_axis);


  // cout << "M_rotated \n";
  // cout << M_rotated;


  point_transform_affine3d tform_inv = inv(tform);
  column_vector M_final = tform_inv(M_rotated);
  return M_final;
}


// DANGER --- this is not under tests.
// Highest prioirty is to write a test for this!
// This computes the change in the x,y,z coordinates of the goal node based on a change
// in the length of the specified edge. Note this is NOT computing the change in score
// (that is, distance to the goal node) but the chnage in the node position.
column_vector Tetrahelix::compute_goal_derivative_c(column_vector cur_coords[],
								int edge_number,
								int goal_node_number) {

  //  cout << "iniit! \n";  
  // This has to be reconstructed....
  column_vector d_e_a(3);


  // SIGH -- this is not using the goal_node_number....
  // Somehow I am not computing anything at all!
  column_vector en = cur_coords[goal_node_number];
  //  cout << "en = " << en << "\n";


  // let P be the joint about which we are rotating by changing e
  // let M be the directly moved joint.
  // let S be the "stable" joint connected to M by e.

  int e = edge_number;
  int s = small_node(e);
  int l  = large_node(e);

  // The edget that is changing is the "CD" edge--opposite AB

  // Note the edge numbering here is a little tricky....
  int c = s;
  int d = l;
  int a = -1;
  int b = -1;
  if (c == l-1) {
    a = l - 3;
    b = l - 2;
  } else if (c == l-2) {
    a = l - 3;
    b = l - 1;
  } else if (c == l-3) {
    a = l - 2;
    b = l - 1;
  }


  //  cout << "edge" << e << " small " << s << " large "  << l << "\n";
  //  cout << "a b c d  " << a << " " << b << " " << c << " " << d << "\n"; 

  column_vector A = cur_coords[a];
  column_vector B = cur_coords[b];
  column_vector C = cur_coords[c];
  column_vector D = cur_coords[d];  

  double ac = distance_3d(A,C);
  double cd = distance_3d(C,D);
  double ad = distance_3d(D,A);        
  double bc = distance_3d(B,C);
  double bd = distance_3d(B,D);
  double ab = distance_3d(A,B);

  // cout << "spud\n";
  // cout << A << "\n";
  // cout << B << "\n";  
  // cout << C << "\n";
  // cout << D << "\n";  

  // cout << "ab ac ad bc bd cd  " << ab << " " << ac << " " << ad << " " << bc  << " " << bd << " "  << cd << "\n";   
    
  double vc = angle_from_three_sides(ad,ab,bd);
  double vd = angle_from_three_sides(ac,ab,bc);
  double vb = angle_from_three_sides(ac,ad,cd);

  //  cout << " vb vc vd  " << vb*180/(M_PI) << " " << vc*180/(M_PI) << " " << vd*180/(M_PI) << "\n";
  
  double dvertex_angle_dlength = dangle_from_dside(ac,ad,cd);
  double ddihedral_dv = ddhedral_dvertex(vd,vc,vb);

  //  cout << " dvertex_angle_dlength  " << dvertex_angle_dlength*180/(M_PI) << "\n";
  //  cout << " ddihedral_dv  " << ddihedral_dv*180/(M_PI) << "\n";

  double dvtheta = dvertex_angle_dlength * ddihedral_dv ;

  //  cout << " dvtheta  " << dvtheta*180/(M_PI) << "\n";  

  // now to compute the vector, we must take dvtheta rotate the vector 
  // around the line A-B the vector of the goal_node

  //  cout << "computing rotaiton: " << dvtheta << " of " << en << "\n";
  //  cout << "about A and B "  <<  "\n";
  //  print_vec(A);
  //  print_vec(B);
  column_vector Emoved = compute_rotation_about_points(A,B,dvtheta,en);
  //  cout << "Emoved " << Emoved << "\n";
  return Emoved - en;
}
