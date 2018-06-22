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

int Tetrahelix::color_of_node(int n) {
  return n % 3;
}

// unfortunately this also has some exceptions (sigh)
bool Tetrahelix::rail_edge_p(int e) {
  if (e == 0) {
    return true;
  } else if (e < 6) {
    return false;
  } else {
    int s = small_node(e);
    int l = large_node(e);
    return color_of_node(s) == color_of_node(l);
  }
}

// This is a unclear at present: I'm trying to distinguish
// between edges that move all further goal nodes as a
// simple hinge from those that require more complicated consideration.
// To some extent this is the same as being an "outside" edge,
// but not the same as being a "rail" edge.
// This is assuming A,B,C are fixed as usual.
bool Tetrahelix::simple_hinge_p(int e) {
  int from_back = num_edges - e;
  return (e < 3) || (from_back == 1) || (from_back == 2) || (from_back == 3) || rail_edge_p(e);
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

    // Note: This is wildly different than the 2d case, which requires just one fixed edge.
    var_edges = num_edges-3;
    node_fixing_order = new int[num_nodes];

    // This is dangerous; I think possibly it should be removed...
    lcoords = new column_vector[num_nodes];      

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
    
    for (int i = 0; i <var_edges; ++i) {
      lower_bound(i) = lower_bound_d;
      upper_bound(i) = upper_bound_d;            
    }

    column_vector coords[3];
    init_fixed_coords_to_z_axis_alignment(coords);
    init_fixed_coords(coords);
    
    Jacobian_memo = new matrix<double>[num_nodes];
    Jacobian_memo_v = new bool[num_nodes];    
}

void Tetrahelix::initialize_Jacobian_memo() {
  for(int i = 0; i < num_nodes; i++) {
    Jacobian_memo_v[i] = false;
  }
}

column_vector fixed[3];
void Tetrahelix::init_fixed_coords(column_vector fcoords[]) {
  fixed[0] = fcoords[0];
  fixed[1] = fcoords[1];
  fixed[2] = fcoords[2];  
}

void Tetrahelix::init_fixed_coords_to_z_axis_alignment(column_vector coords[]) {

    double f = 3;
    column_vector temp0(3);
    // These values are taken from: https://pubinv.github.io/tetrahelix/
    temp0 = 0.25980762113533157,  0.7794228634059948,  -1;
    coords[0] = temp0*f;

    temp0 =  -0.17320508075688773, 0.9730720307163656, -0.841886116991581;   
    coords[1] = temp0*f;
    
    temp0 =  -0.028867513459481214,  0.5212239736588337, -0.683772233983162;
    coords[2] = temp0*f;
    fixed[0] = coords[0];
    fixed[1] = coords[1];
    fixed[2] = coords[2];
}

void Tetrahelix::restore_fixed_coords(column_vector coords[]) {
  coords[0] = fixed[0];
  coords[1] = fixed[1];
  coords[2] = fixed[2];  
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
      pow(sqrt(an) - dad,2) +
      pow(sqrt(bn) - dbd,2) +
      pow(sqrt(cn) - dcd,2);
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
column_vector find_point_from_transformed(Chirality sense,double AB, double AC, double AD, double BC, double BD, double CD, bool* valid) {
  // _m2 means "squared"
  double AB_m2 = AB * AB; double AC_m2 = AC * AC;
  double AD_m2 = AD * AD; double BC_m2 = BC * BC;
  double BD_m2 = BD * BD; double CD_m2 = CD * CD;
  double qx = AB;
  double rx = (AB_m2 + AC_m2 - BC_m2) / (2.0 * AB);    
  double ry = sqrt (AC_m2 - rx * rx);
  double sx = (AB_m2 + AD_m2 - BD_m2) / (2.0 * AB);
  double sy = (BD_m2 - (sx - qx) * (sx - qx) - CD_m2 + (sx - rx) * (sx - rx) + ry * ry) / (2 * ry);
  double factor = AD_m2 - sx * sx - sy * sy;
  double sz = 0;
  int debug = 0;
  if (factor < 0) {
    if (debug) {
      cout << "INTERNAL ERROR: this is not a legal tetrahedron\n";
      cout << "AB AC AD BC BD CD\n";
      cout << AB << " " << AC << " " << AD << " " << BC << " " << BD << " " << CD << "\n";
    }
    // I have no idea what to return here -- I'll return half the average distance.
    sz = (AB + AC + AD + BC + BD + CD) / (6*2);
    *valid = false;
  } else {
    sz = sqrt (AD_m2 - sx * sx - sy * sy);
    *valid = true;
  }
    column_vector A(3);
    column_vector B(3);
    column_vector C(3);     
    A = 0.0,0.0,0.0;
    B = qx,0.0,0.0;
    column_vector D(3);
    D = sx,sy,(sense == CCW) ? sz : -sz;
    // We compute this only for debugging purposesn
    C = rx,ry,0.0;

    if (debug) {
      Chirality senseABC = tet_chirality(A,B,C,D);
      cout << "chirality (demanded, computed) " << sense << ", " << senseABC << "\n";
      if (sense != senseABC) {
	cout << "SENSE CHECK ON TET FAILED!\n";
	print_vec(A);
	print_vec(B);
	print_vec(C);
	print_vec(D);
      }
      assert(sense == senseABC);	      
    }
    return D;

}


// This is a slighlty different way based on answer by Kuba Ober in ths same way.
// This tries to rotate the vector A-B on to the X-Axis. C should end up in the XY plane.
// I think if the the vector A-B points in the negative x direction, we have a special case
// that causes a problem here. In that case we are 180 degrees out of sync. I need to think
// carefully about how to handle this.
point_transform_affine3d compute_transform_to_axes2(column_vector pA, column_vector pB, column_vector pC) {
  int debug = 0;
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
  if (debug)  {  
   cout << "Cn\n";
   print_vec(Cn);

    
   cout << "Cn(2) " << Cn(2) << "\n";
   cout << Cn(0)*Cn(0) + Cn(1)*Cn(1) << "\n";
  }

  double len = alignX(pC).length();  
  double denom = sqrt(Cn(0)*Cn(0) + Cn(1)*Cn(1));
  if (debug) {
   cout << "len " << len << "\n";
   cout << "denom: " << denom << "\n";
   cout << Cn(0)*Cn(0) + Cn(1)*Cn(1) << "\n";
  }
  
  double value = Cn(2)/denom;

  if (debug) {
    cout << "value " << value << "\n";
    cout << "near zero " << abs(value - 1.0) << "\n";
  }
  double theta;

  theta = -atan2(Cn(2),Cn(1));

  if (debug) cout << "theta: " << theta*180/(M_PI) << "\n";      

  // // The big question here is does this transform in some since reverse chirality?
  // // Is there a way for us to test that this maintains chirality?
  // if (abs(value - 1.0) < 1e-3) {
  //    cout << "UNITARY\n";
  //     theta = -M_PI/2;
  //   } else {
  //   // how do we know this is the correct sign?
  //   if (debug) cout << "Cn(2),Cn(1) " << Cn(2) << ", " << Cn(1) << "\n";

  //   // I actually think this value needs to be chosen so that the transform does not change
  //   // the chirality of a point from the triangle ABC. This is not obvious to me.
  //   // We can obviously make two choises on how to rotate...
  //   // These two are opposite directions because atan2(x,y) + atan2(y,x) = PI/2.
  //    theta = atan2(Cn(1),Cn(2));
  //   //  theta = -atan2(Cn(2),Cn(1));    
  //   if (debug) cout << "theta: " << theta*180/(M_PI) << "\n";    
  // }

  // Now, if theta is a multiple of 180 degrees, we don't really need to rotate...
  point_transform_affine3d result;
  if ((((int) round(theta*180/(M_PI))) % 180) == 0) {
    // cout << "Yes, this happened to be a 180 rotation, so avoiding!\n";
    // cout << "pc " << pC << "\n";
    // cout << alignX(pC) << "\n";
    // cout << alignX.get_m() << "\n";
    // cout << alignX.get_b() << "\n";  
    result = rotate_around_x(0) *alignX;    
  } else {
    result = rotate_around_x(theta) * alignX;
  }

  if (debug) {
    cout << "These should be aligned... \n";
    column_vector lA = result(pA);
    column_vector lB = result(pB);
    column_vector lC = result(pC);    
    
    print_vec(lA);
    print_vec(lB);
    print_vec(lC);        
  }
  
  return result;
}

column_vector find_fourth_point_given_three_points_and_three_distances(Chirality sense,
								      column_vector pa,column_vector pb,column_vector pc,
								       double ad,double bd,double cd,
								       bool* valid
								      ) {

  int debug = 0;
  if (debug) cout << "ad bd cd\n";
  if (debug) cout << ad << " " << bd << " " << cd << "\n";
  
  // First compute all 6 distances....
  double ab = distance_3d(pa,pb);
  double ac = distance_3d(pa,pc);
  double bc = distance_3d(pb,pc);
  
  // Now find transformation that rotates and translates to axes...
  if (debug) cout << "pa pb pc \n";
  if (debug) cout << pa << " " << pb << " " << pc << "\n";  
  point_transform_affine3d tform = compute_transform_to_axes2(pa,pb,pc);

  column_vector Ap(3);
  column_vector Bp(3);
  column_vector Cp(3);

  Ap = tform(pa);
  if (debug) cout << "Ap \n";
  if (debug) print_vec(Ap);
  
  Bp = tform(pb);
  if (debug) cout << "Bp \n";
  if (debug) print_vec(Bp);
  
  Cp = tform(pc);
  if (debug) cout << "Cp \n";
  if (debug) print_vec(Cp);
  
  point_transform_affine3d tform_inv = inv(tform);
  if (debug) cout << "tform inv\n";
  if (debug) cout << tform_inv.get_m();

  // Now get the fourth point...
  if (debug) cout << "INPUT YYY\n";
  if (debug) cout << ab << " " << ac << " " << ad << " " << bc << " " << bd << " " << cd << "\n";
  column_vector D = find_point_from_transformed(sense,ab,ac,ad,bc,bd,cd,valid);
  
  if (isnan(D(0)) || isnan(D(1)) || isnan(D(2))) {
    cout << "INPUT YYY\n";
    cout << D(0) << "\n";
    cout << D(1) << "\n";
    cout << D(2) << "\n";        
    cout << ab << " " << ac << " " << ad << " " << bc << " " << bd << " " << cd << "\n";
  }
  // assert(!isnan(D(0)));
  // assert(!isnan(D(1)));
  // assert(!isnan(D(2)));

  Chirality untransformed = tet_chirality(pa,pb,pc,tform_inv(D));
  Chirality transformed = tet_chirality(Ap,Bp,Cp,D);
  if (debug) {
    cout << "Chirality demanded: " << sense << "\n";  
    cout << "Chirality of transformed computations: " << transformed << "\n";
    cout << "Chirality of untransformed computations: " << untransformed << "\n";
  }
  return tform_inv(D);
}



// This routine is one of the main ways the coords change;
// it therefore has the effect of invalidating existing Jacobians by virtue
// of changing the computation.
bool TetrahelixConfiguration::forward_find_coords() {
    FindCoords3d f;

    int debug = 0;
    //    Chirality sense = CCW;
    Chirality sense = CCW;
    // Is this even required?
    thlx->restore_fixed_coords(coords);

    if (debug) {
       cout << "in solve distances:\n";
       for(int i = 0; i < thlx->num_edges; i++) {
         cout << "i, d " << i << " , " << thlx->distance(i) << "\n";
       }

       cout << "SOLVE_FORWARD_FIND_COORDS \n";
       for(int i = 0; i < thlx->num_nodes; i++) {
         print_vec(coords[i]);
       }
    }
    
    // Basic structure: Iteratively find coordinates based on simple triangulations.
    // This only works for actuator networks in which we can
    int fs = thlx->fixed_nodes.size();
    //    cout << "fixed nodes " << fs << "\n";
    bool valid = true;
    for(int i = fs; i < thlx->num_nodes; i++) {
      // Let's set up initial values
      // choose a starting point

      f.a.set_size(3);
      f.a = coords[i-fs];

      f.b.set_size(3);
      f.b = coords[i-(fs-1)];

      f.c.set_size(3);
      f.c = coords[i-(fs-2)];

      // cout << "f.a f.b f.c\n";
      // print_vec(f.a);
      // print_vec(f.b);
      // print_vec(f.c);

      f.chi = sense;

      // and minimize the function
      if (debug_find) std::cout << "f.a " << f.a << std::endl;
      if (debug_find) std::cout << "f.b " << f.b << std::endl;
      if (debug_find) std::cout << "f.c " << f.c << std::endl;      

      int ecd = thlx->edge_between(i,i-1);
      int ebd = thlx->edge_between(i,i-2);
      int ead = thlx->edge_between(i,i-3);
      if (debug) {
	cout << "ecd ebd ead \n";
	cout << ecd << " " << ebd << " " << ead << "\n";
      }

      f.dad = thlx->distance(ead);
      f.dbd = thlx->distance(ebd);
      f.dcd = thlx->distance(ecd);

      column_vector  y(3);

      if (debug) {
	cout << "input XXXX (chirality) " << f.chi << "\n";

	cout << f.a << "\n";
	cout << f.a << "\n";
	
	cout << f.b << "\n";
	cout << f.c << "\n";
	cout << "=======\n";
	cout << f.dad << "\n";
	cout << f.dbd << "\n";
	cout << f.dcd << "\n";
      }
      bool lvalid = true;
      y = find_fourth_point_given_three_points_and_three_distances(f.chi,f.a,f.b,f.c,
								   f.dad,f.dbd,f.dcd,&lvalid);
      valid &= lvalid;

      
      coords[i].set_size(3);
      coords[i] = y;
      if (debug) {
	cout << "SETTING " << i << "\n";
	print_vec(y);
	cout << "valid?: " << valid << "\n";
	cout << " distances \n";
	for (int i = 0; i < thlx->num_nodes-1; ++i) {
	  for (int j = i+1; j <  thlx->num_nodes; ++j) {    
	    cout << " i,j : " << i << "," << j << " ";
	    cout << distance_3d(coords[i],coords[j]) << "\n";
	  }
	}
	
      }
      if (debug_find) std::cout << "f(y) " << f(y) << std::endl;
    }


    if (debug) {
       cout << "FINAL COORDS \n";
       for(int i = 0; i < thlx->num_nodes; i++) {
         print_vec(coords[i]);
       }
    }
    
    return valid;
};

void Tetrahelix::add_goal_node(int num,double x,double y, double z,double w)    {
      column_vector mg(3);
      mg(0) = x;
      mg(1) = y;
      mg(2) = z;
      goals.push_back(mg);
      goal_nodes.push_back(num);
      goal_weights.push_back(w);
      //      cout << "QQQ " << goals.size() << "\n";
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

// This is the two dimension angle of a triangle from lengths
// based on triangle law.
double cosine_from_three_sides(double adjacent1, double adjacent2, double opposite) {
  double a = adjacent1;
  double b = adjacent2;
  double c = opposite;
  return (a*a + b*b - c*c)/(2*a*b);
}

double angle_from_three_sides(double adjacent1, double adjacent2, double opposite) {
  return acos(cosine_from_three_sides(adjacent1, adjacent2, opposite));
}
// This is change in the angle c wrt to a change to the length DC given
// lengthe a, b (adjajent to ANGLE ACB and c (the opposite length).
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

// This is the change in the dihedral angle wrappe around  with respect to
// the vertex angles AT THE TIP.  So if A is the tip, pointing to B, C, D,
// the the dihedral angle across AB based on the tip angles BAD (adjacent1)
// and BAD (adjacent2)  and CAD (opposite) is geiven by this function.
double ddhedral_dvertex(double adjacent1, double adjacent2, double opposite) {
  double r = adjacent1;
  double t = adjacent2;
  double s = opposite;
  double num = csc(r)*csc(t)*sin(s);
  double den = sqrt(1 - csc(r)*csc(r) * csc(t)*csc(t) * pow( cos(s) - cos(r)*cos(t),2));
  return num/den;
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

// This is differential calculator for the purpose of testing the derivatives.
// We change the specified edge by the differential "delta" and compute
// the change in the position of the goal node as report that as an
// approximation of the derivative. Possibly this could even be
// used as an estimate of the deriviative, although it is embarassing to me
// that I am having so much trouble figuring it out analytically, but the
// truth is I am. Possibly figuring out the derivative in all cases
// would be considered a "starred" problem in a textbook. I doubt seriously
// it is an open problem, however.
column_vector Tetrahelix::compute_goal_differential_c(column_vector cur_coords[],
								int edge_number,
						      int goal_node_number,
						      double delta_fraction) {
  TetrahelixConfiguration thc(this,cur_coords);
  set_distances(cur_coords);
  thc.forward_find_coords();
  
  column_vector g_orig = cur_coords[goal_node_number];
  double orig_length = distance(edge_number);
  distance(edge_number) += delta_fraction * distance(edge_number);

  thc.forward_find_coords();
  
  column_vector differential = (cur_coords[goal_node_number] - g_orig)/delta_fraction;
  distance(edge_number) = orig_length;
  
  thc.forward_find_coords();  

  return differential;
}

// This computes the change in the x,y,z coordinates of the goal node based on a change
// in the length of the specified edge. Note this is NOT computing the change in score
// (that is, distance to the goal node) but the chnage in the node position.
column_vector Tetrahelix::compute_goal_derivative_c(column_vector cur_coords[],
								int edge_number,
								int goal_node_number) {


  int debug = 0;
  //  if (edge_number == 5) debug = 1;
  if (debug) cout << "INIT COMPUTE_GOAL_DERIVATIVE\n";
  //  cout << "iniit! \n";  
  // This has to be reconstructed....
  column_vector d_e_a(3);


  column_vector en = cur_coords[goal_node_number];

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


  if (debug)  cout << "edge" << e << " small " << s << " large "  << l << "\n";
  if (debug)  cout << "a b c d  " << a << " " << b << " " << c << " " << d << "\n"; 

  // How does this choose the chirality? Which side of the triangle BCD does A occur on?
  // If we look at the tip of A the BCD is on the other side (from our eye), does BCD go
  // clockwise or anticlockwise? Is our convention that, starting from the fixed points,
  // the normal of ABC points toward or away from D?
  column_vector A = cur_coords[a];
  column_vector B = cur_coords[b];
  column_vector C = cur_coords[c];
  column_vector D = cur_coords[d];

  //  debug = 0;
  Chirality tet = tet_chirality(A,B,C,D);
  if (tet != CCW ) {
    if (debug) {
      cout << "Non-anti-clockwise tet detected: A B C D\n";
      print_vec(A);
      print_vec(B);
      print_vec(C);
      print_vec(D);
      cout << "\n";
      cout << "swapping A and B to repair\n";
    }
    column_vector S = B;
    B = A;
    A = S;
    tet = tet_chirality(A,B,C,D);
  }

  if (tet != CCW) {
    // This is a major internal error...
    cout << "Internal error. Could not construct CCW tet via a swap.\n";
      cout << "Non-anti-clockwise tet detected: A B C D\n";
      print_vec(A);
      print_vec(B);
      print_vec(C);
      print_vec(D);
      cout << "\n";
    
    assert(tet == CCW);    
  }


  double ac = distance_3d(A,C);
  double cd = distance_3d(C,D);
  double ad = distance_3d(D,A);        
  double bc = distance_3d(B,C);
  double bd = distance_3d(B,D);
  double ab = distance_3d(A,B);

  if (debug) {
    cout << "A B C D\n";
    print_vec(A);
    print_vec(B);
    print_vec(C);
    print_vec(D);
    cout << "\n";
  }


  // cout << "ab ac ad bc bd cd  " << ab << " " << ac << " " << ad << " " << bc  << " " << bd << " "  << cd << "\n";   
  // Are these properly tested?  I have reason to believe this may not be right!!!
  // These are supposed to be the interior strap angles...
  // I believe these are supposed by two-dimensional angles for simple triangles..
  // But the are not named corretly!!!
  // Naming should be adjacen-center-adjacent..
  double ang_BAD = angle_from_three_sides(ad,ab,bd);
  double ang_BAC = angle_from_three_sides(ac,ab,bc);
  double ang_CAD = angle_from_three_sides(ac,ad,cd);

  if (debug) cout << " BAD BAC CAD  " << ang_BAD*180/(M_PI) << " " << ang_BAC*180/(M_PI) << " " << ang_CAD*180/(M_PI) << "\n";

  // This is the change in the vertex angle c (opposite CD).
  double dvertex_angle_CAD = dangle_from_dside(ac,ad,cd);


  // This the change in the dihedral angle AB based on the angles of the triangles meeting at A.
  // That is, it is the change of the dihedral angle around AB looking from A to B.
  // Note: Positive means anticlockwise.
  double ddihedral_AB = ddhedral_dvertex(ang_BAD,ang_BAC,ang_CAD);

  //  debug = 0;
  if (debug) {
   cout << " dvertex_angle_dlength  " << dvertex_angle_CAD*180/(M_PI) << "\n";
   cout << " ddihedral_dv  " << ddihedral_AB*180/(M_PI) << "\n";
  }

  // This is the change in the dihedral angle AB with respect to the change in length CD
  double dAB_LENCD = dvertex_angle_CAD * ddihedral_AB ;
    // because we use this as the INTERIOR angle, the change in the
    // interior is the negation of the EXTERIOR, so we change the sign

    if (debug) {
      cout << " dAB_LENCD  " << dAB_LENCD*180/(M_PI) << "\n";  

  // now to compute the vector, we must take dvtheta rotate the vector 
  // around the line A-B the vector of the goal_node

  //   cout << "computing rotaiton: " << dvtheta*180/(M_PI) << " of " << en << "\n";
     cout << "about A and B "  <<  "\n";
     print_vec(A);
     print_vec(B);
    }

    // G is the goal direction, the vector which if added to D would place us at the goal.
    // WARNING: I don't understand this line:
    column_vector G = en - A;
    column_vector BA = A - B;
    // What order should this be?
    column_vector CP = cross_product(G,BA);
    column_vector deriv = CP*dAB_LENCD;
    deriv = deriv/l2_norm(deriv);
    if (debug) {
      cout << "G = ";
      print_vec(G);
      cout << "\n";
      
      cout << "CP \n";
      print_vec(CP);
    }

  // I now suspect this is rotating in the wrong direction! Or that I mus
  // at least be careful of the order!
    //  column_vector Emoved = compute_rotation_about_points(A,B,dAB_LENCD,en);
    //  column_vector differential = Emoved - en;  
    //  debug = 0;  
  if (debug) {
    //    cout << "differential ";
    //    print_vec(differential);
    cout << "deriv ";
    print_vec(deriv);  
  }
  //  return Emoved - en;
  //  return differential;
  return deriv;
}


// WAR?NING: There is an assumption here which will mess us up when
// it comes to computing multiple goals.  The Jacobian is only for the goal_node.
// At present I am keeping one such Jacobian, which is probably not right!
// To be able to compute for any node, I really need an array of Jacobians.
column_vector Tetrahelix::compute_goal_derivative_j(column_vector cur_coords[],
								int var_edge_number,
								int goal_number) {
  column_vector edge_length_deriv(var_edges);
  for(int i = 0; i < var_edges; i++) {
    edge_length_deriv(i) = 0.0;
  }
  assert(var_edge_number < var_edges);
  edge_length_deriv(var_edge_number) = 1.0;
  matrix<double> deriv_i = normalize(Jacobian_temp * edge_length_deriv);  
  return deriv_i;
}

// This is the deriviative of the cosine wrt the first argument
double Tetrahelix::dcos_adj(double adj1, double adj2, double opp) {
  double z = adj1;
  double q = adj2;
  double s = opp;
  return (z*z + -q*q + s*s) / (2*z*z*q);
}

// This is an preliminiary attempt to test the
double Tetrahelix::d_dihedralBC_dCD_aux(double BC, double BD, double CD, double CE, double DE, double aBCE) {
  double a = aBCE;

  // f = cos DCE
  double f = cosine_from_three_sides(CD,CE,DE); 
  // g = cos BCD
  double g = cosine_from_three_sides(CD,BC,BD);

  double fp = dcos_adj(CD,CE,DE);
  double gp = dcos_adj(CD,BC,BD);
  
  double sa = sin(a);
  double ca = cos(a);

  // Got 1 - g is negative!
  double t1 = sa*(fp - ca*gp)/sqrt(1 - g*g);
  double t2 = sa*g*gp*(f - ca*g);
  double t2den = pow((1 - g*g),3.0/2.0);
  
  double num = t1 + t2/t2den;
  double n1 = pow(sa*(f - ca*g),2);
  double den = sqrt(1 - (n1/(1-g*g)));

  //  cout << "internal debug values:\n";
  //  cout << "num den n1 t1 t2:\n";
  //  cout << num << " " << den << " " << n1 << " " << t1 << " " << t2 << "\n";
  
  return num/den;
}

// The computation of "e" here is too simple.
// it is not computing it correctly.
double Tetrahelix::d_dihedralBC_dCD(column_vector cur_coords[],
			   int edge_number) {
  int debug = 0;
  if (debug) cout << "d_dihedralBC_dCD\n";

  // This has to be reconstructed....
  column_vector d_e_a(3);

  // let P be the joint about which we are rotating by changing e
  // let M be the directly moved joint.
  // let S be the "stable" joint connected to M by e.
  int e_n = edge_number;
  int s = small_node(e_n);
  int l  = large_node(e_n);
  if (l + 1 > num_nodes) {
    cout << "edge number too high!\n";
    cout << "abort!";
    abort();
  }

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
  int e = d+1;

  if (debug)  cout << "edge" << e << " small " << s << " large "  << l << "\n";
  if (debug)  cout << "a b c d  e" << a << " " << b << " " << c << " " << d << " " << e << "\n"; 

  // How does this choose the chirality? Which side of the triangle BCD does A occur on?
  // If we look at the tip of A the BCD is on the other side (from our eye), does BCD go
  // clockwise or anticlockwise? Is our convention that, starting from the fixed points,
  // the normal of ABC points toward or away from D?
  column_vector A = cur_coords[a];
  column_vector B = cur_coords[b];
  column_vector C = cur_coords[c];
  column_vector D = cur_coords[d];
  column_vector E = cur_coords[e];

  double bc = distance_3d(B,C);
  double bd = distance_3d(B,D);
  double be = distance_3d(B,E);  
  double cd = distance_3d(C,D);        
  double ce = distance_3d(C,E);
  double de = distance_3d(D,E);

  //  if (debug)  cout << "bc bd be cd ce de" << bc << " " << bd << " " << be << " " << cd << " " << ce << " " << de << "\n";   

  double ang_BCE = angle_from_three_sides(bc,ce,be);

  if (debug)
    cout << "Ang : " << ang_BCE*180/M_PI << "\n";

  // I'm worried that this is not signed!! Is it always positive? If so, how can it represent both counter clockwise and
  // clockwise rotations! I think this must depend on the chirality of BC somehow!
  // Which side of BCD is E on?
  Chirality tet = tet_chirality(B,C,D,E);

  double d_dihedral = d_dihedralBC_dCD_aux(bc, bd, cd, ce, de, ang_BCE);

  if (debug)
    cout << "TET : " << tet << "\n";
  return (tet == CCW) ? d_dihedral : -d_dihedral;
}

// MAJOR WARNING: There is something wrong with my sign here.
// Either I am not signing the dihedral correctly, or I am making some other mistake.
// I am having BC change directions, but the dihedral calculations does not change!
// WARNING: this is using the node E (not using goal_number)
column_vector Tetrahelix::compute_goal_derivative_after_edge_internal(column_vector cur_coords[],
								int edge_number,
								int goal_node_number) {
  int debug = 0;

  if (debug) cout << "d_dihedralBC_dCD\n";

  // let P be the joint about which we are rotating by changing e
  // let M be the directly moved joint.
  // let S be the "stable" joint connected to M by e.
  int e_n = edge_number;
  int s = small_node(e_n);
  int l  = large_node(e_n);
  if (l + 1 > num_nodes) {
    cout << "edge number too high!\n";
    cout << "abort!";
    abort();
  }

  // The edge that is changing is the "CD" edge--opposite AB
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
  int e = d+1;

  if (debug)  cout << "edge" << e_n << " small " << s << " large "  << l << "\n";
  if (debug)  cout << "a b c d  e\n" << a << " " << b << " " << c << " " << d << " " << e << "\n"; 

  // How does this choose the chirality? Which side of the triangle BCD does A occur on?
  // If we look at the tip of A the BCD is on the other side (from our eye), does BCD go
  // clockwise or anticlockwise? Is our convention that, starting from the fixed points,
  // the normal of ABC points toward or away from D?
  column_vector A = cur_coords[a];
  column_vector B = cur_coords[b];
  column_vector C = cur_coords[c];
  column_vector D = cur_coords[d];
  column_vector E = cur_coords[e];

  double d_BC = d_dihedralBC_dCD(cur_coords,
			    edge_number);

  // Possibly E here should be coords[goal_number] instead; that would
  // work for something rigidly attached to E relative to BC.

  column_vector C_to_E = E-C;
  column_vector B_to_C = C-B;
  if (debug) {
    cout << "C to E\n";
    print_vec(C_to_E);
    cout << "B to C\n";  
    print_vec(B_to_C);
  }
  column_vector cp = cross_product(B_to_C,C_to_E);
  if (debug) {
    cout << "cross product\n";
    print_vec(cp);
  }
  
  column_vector dE_dl = cp * d_BC;
  return dE_dl;
}
		   
// TODO: Add number of fixed nodes as a definite function and replace all
// references to naked literals that depend on it.

void Tetrahelix::set_distances(column_vector coords[]) {
    for (int i = 0; i < num_edges; ++i) {
      //      int e = edge_number_of_nth_variable_edge(i);
      int s = small_node(i);
      int l  = large_node(i);
      double d = distance_3d(coords[s],coords[l]);
      //      cout << "i s l d \n";
      //      cout << i << " " << " " << s << " " << l << " " << d << "\n";
      distance(i) = d;
    }
    
}

void set_row_from_vector(matrix<double> *M, column_vector c, int row, int col, int n) {
  for(int i = 0; i < n; i++) {
    (*M)(row,col+i) = c(i);
  }
}

void stack_9x3_matrix(matrix<double> *JK,matrix<double> J1,matrix<double> J2,matrix<double> J3) {
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {    
      (*JK)(i,j) = J1(i,j);
    }
  }
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {    
      (*JK)(i+3,j) = J2(i,j);
    }
  }
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {    
      (*JK)(i+6,j) = J3(i,j);
    }
  }
}

// This is my attempt to follow the Lee-Sanderson approach to computing the
// Jacobian, which I find rather daunting. For the purpose of testing,
// I am considering only a 5-node tet. This fixes the number of variable struts at 6,
// an important aspect of the dimensionality of Jacobians.
// The Jacobian is there for a 3 x 6 matrix.
matrix<double> Tetrahelix::JacobianBase(column_vector coords[]) {
  // First I will attempt to construct Btet.

  matrix<double> Btet(3,3);

  column_vector DA = normalize(coords[3] - coords[0]);
  column_vector DB = normalize(coords[3] - coords[1]);
  column_vector DC = normalize(coords[3] - coords[2]);

  set_row_from_vector(&Btet,DA,0,0,3);
  set_row_from_vector(&Btet,DB,1,0,3);
  set_row_from_vector(&Btet,DC,2,0,3);

  int debug = 0;

  if (debug) {
    cout << "Btet \n";
    cout << Btet;
    cout << "End Btet\n";
  }
  
  
  matrix<double> Btet_inv = inv(Btet);

  if (debug) {  
    cout << "Btet_inv \n";
    cout << Btet_inv;
    cout << "End Btet_inv\n";
  }
  
  // now somehow I have to construct a 3 x 9 matrix with zero columns...
  // Since the base is the node numbered 3, stuts 0,1,2, 6,7,8, and 9
  // should be zero...this is very error prone.
  matrix<double> J3 = zeros_matrix<double>(3,var_edges);
  // Now, hopefully we can lay Btet_inv right in as 3,4,5....
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {    
      J3(i,j) = Btet_inv(i,j);
    }
  }
  return J3;
}

// This is my attempt to follow the Lee-Sanderson approach to computing
// a Jacobian.
// The idea is to compute the Jacobian node "node" from the coords.
// At present this is recursive operation.
// In a sense, J5 is the ultimate, because it includes all other
// Jacobians, though all Jacobians technically map changes strut lengths
// to the change in position: That is, all Jacobian are shaped as 3x9 matrices
// for a 5-node tetrobot. However, it is not valid to compute the
// change in node 5 with J4. However, if you only need the change in node
// 4, you could stop at J4.
// Note that this is dangerously mixing my numbering scheme with
// the Lee-Anderson numbering scheme.

void Tetrahelix::fill_Jacobian(column_vector coords[]) {
  for(int i = 0; i < num_nodes; i++) {
    Jacobian_memo[i] = Jacobian(coords,i);
    Jacobian_memo_v[i] = true;
  }
}

matrix<double> Tetrahelix::get_Jacobian(column_vector coords[],int node) {
  if (!Jacobian_memo_v[node]) {
    cout << "INTERNAL ERROR: Jacobians not set up properly!\n";
    abort();
  } else {
    return Jacobian_memo[node];
  }
}

matrix<double> Tetrahelix::Jacobian(column_vector coords[],int node) {
  // First I will attempt to construct Btet.
  if (node < 3) {
    return zeros_matrix<double>(3,var_edges);
  } else if (node == 3) {
    return JacobianBase(coords);
  } else {
    // Now here we are supposed to "stack" (multiply the Jacobians).
    // Each of these will have the shape 3 x 9 (for a 5-tet).
    // Note that this should be memoized, which is actually very important.
    matrix<double> J1 = get_Jacobian(coords,node-1);
    matrix<double> J2 = get_Jacobian(coords,node-2);
    matrix<double> J3 = get_Jacobian(coords,node-3);
    //    cout << "Got all memos!";
  
    int n = node;
  
    column_vector DA = normalize(coords[n] - coords[n-3]);
    column_vector DB = normalize(coords[n] - coords[n-2]);
    column_vector DC = normalize(coords[n] - coords[n-1]);

    matrix<double> Btet(3,3);
    
    set_row_from_vector(&Btet,DA,0,0,3);
    set_row_from_vector(&Btet,DB,1,0,3);
    set_row_from_vector(&Btet,DC,2,0,3);
  
    matrix<double> Atet = zeros_matrix<double>(3,9);

    int debug = 0;
    if (debug) {
      cout << "Atet \n";
      cout << Atet;
      cout << "End Atet\n";
    }
  
    set_row_from_vector(&Atet,-DA,0,0,3);
    set_row_from_vector(&Atet,-DB,1,3,3);
    set_row_from_vector(&Atet,-DC,2,6,3);
  
    matrix<double> Btet_inv = inv(Btet);


    if (debug) {
      cout << "Atet \n";
      cout << Atet;
      cout << "End Atet\n";
    }

    //    matrix<double> JK = zeros_matrix<double>(9,3);
    matrix<double> JK = join_cols(join_cols(J3,J2),J1);

    if (debug) {    
      cout << "JK \n";
      cout << JK;
      cout << "JK\n";
  
      cout << "part1 \n";
      cout << Atet * JK;
      cout << "part1\n";

      cout << "part2 \n";
      cout << (Btet_inv*(Atet*JK));
      cout << "part2\n";

    
      cout << "Btet_inv \n";
      cout << Btet_inv;
      cout << "End Btet_inv\n";
    }

    // according to the paper, Ctet is Btet_inv with zero columns added...
    // not entirely sure how that will work.
    matrix<double> Ctet = zeros_matrix<double>(3,var_edges);

    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
	Ctet(i,j+3*(node - 3)) = Btet_inv(i,j);
      }
    }
    matrix<double> Ju = Ctet - (Btet_inv*(Atet*JK));

    if (debug) {        
      cout << "Ju \n";
      cout << Ju;
      cout << "Ju\n";
    }
  
    return Ju;
  }
}

TetrahelixConfiguration::TetrahelixConfiguration(Tetrahelix* thlx,column_vector *coords)
{
  this->thlx = thlx;
  this->coords = coords;
}


// WARNING: For now we are only supporting the end-goal effector with our Jacobian,
// but later this will have to be more sophisticated.
// What we really need to do now is to store one Jacobian for each goal node.
// These will have to be updated here.
void TetrahelixConfiguration::declare_coord_changed(int n) {
  forward_find_coords();
  clock_t t;
  t = clock();	  
  matrix<double> Ju = thlx->Jacobian(coords,thlx->num_nodes-1);
  t = clock() - t;	
  time_in_jacobian += t;
  int debug = 0;
  if (debug) {
    cout << "Jacobian:\n";
    cout << Ju;
    cout << "End Jacobian\n";
    cout << "trying Ju\n";
  }
  thlx->Jacobian_temp = Ju;
}
