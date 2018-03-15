// derivative3d.cpp -- Test of derivative of tetrahedron
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

#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>

#include <math.h>

#define BOOST_TEST_MODULE My3DTest
#include <boost/test/unit_test.hpp>

#include "ActNetUtility.hpp"
#include "Tetrahelix.hpp"

using namespace dlib;
using namespace std;


#define TRUSS_NODES 4
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5




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

BOOST_AUTO_TEST_CASE( test_edge_numbering )
{
  Tetrahelix thlx(TRUSS_NODES+3,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL);
  //  cout << "num_nodes: " << thlx.num_nodes << "\n";
  //  cout << "num_edges: " << thlx.num_edges << "\n";  
  
  for(int i = 0; i < thlx.num_edges; i++) {
    int sm = thlx.small_node(i);    
    int ln = thlx.large_node(i);
    //    cout << "i, small, large\n" << i << " " << sm << "," << ln << "\n";
    
    int e0 = thlx.edge_between(sm,ln);

    //    cout << "edge in, edge computed\n";
    //    cout << i << " == " << e0 << "\n";
    BOOST_CHECK( i == e0 );
  }
}

BOOST_AUTO_TEST_CASE( test_distance )
{
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);  
  
  // Please see accompanying diagram.
  // C_z = 0
  // B_x = 0, B_y = 0
  // C_x = 0, C_y = 0
  // D_y = 0

  // now for test purposes I will choose some other values.
  double theta = 60.0 * M_PI/ 180.0; // this is 45 degrees.
  double Cx = cos(theta);
  double Cy = sin(theta);
  A = 0.0,0.0,1.0;
  B = 0.0,0.0,-2.0;
  C = Cx,Cy,0.0;
  D = 1.0,0.0,0.0;
  //  cout << "A" << A << "\n";
  //  cout << "C" << C << "\n";
  //  cout << "A C " <<  distance_3d(A,C) << "\n";
  //  cout << "A B " <<  distance_3d(A,B) << "\n";    
}
// Basically we want to construct a special tetrahedron
// aligned on the axes such that the computation
// of the change of angle as a change a length is quite
// easy. This will be used to test more general approaches
// to finding the derivative.
BOOST_AUTO_TEST_CASE( test_3Dtet )
{
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);  
  
  // Please see accompanying diagram.
  // C_z = 0
  // B_x = 0, B_y = 0
  // C_x = 0, C_y = 0
  // D_y = 0

  // now for test purposes I will choose some other values.
  double theta = 60.0 * M_PI/ 180.0; // this is 45 degrees.
  double Cx = cos(theta);
  double Cy = sin(theta);
  A = 0.0,0.0,1.0;
  B = 0.0,0.0,-2.0;
  C = Cx,Cy,0.0;
  D = 1.0,0.0,0.0;

  // now we have a particular tetrahedron.
  // We want to compute d theta/ dl, where L = len(DC).
  double epsilon = 1.0 * M_PI/ 180.0;

  // what I really need to do is to keep this, and then
  // change the length by leaving D alone but solving for the new
  // theta, and see if it really matches the differential.
  // That is, check that the differential matches the derivative.
  for(int i = 0; i < 40; i++) {
    D(0) = (1.0 + i*0.1);
    double dist = distance_3d(C,D);

    double dtheta = dtheta_dd_laid_on_axes(dist,D(0),D(2));
    double dtheta_tenth = 0.1 * dtheta;
    //    cout << i << " " << dist << " " << dtheta*180.0/M_PI << "\n";
    //    cout << i << " " << dist << " " << dtheta_tenth*180.0/M_PI << "\n";

    //    double ctheta = atan2(C(1),C(0));
    //    cout << "ctheta " << ctheta*180.0/M_PI << "\n";    
    double xtheta = acos((-dist*dist + D(0)*D(0) + D(2)*D(2) +1.0)/(2.0*D(0)));
    //    cout << "xtheta " << xtheta*180.0/M_PI << "\n";
    double ytheta = acos((-(dist+0.1)*(dist+0.1) + D(0)*D(0) + D(2)*D(2) +1.0)/(2.0*D(0))); 


    // let b = the triangle opposite point B.

    double ac = distance_3d(A,C);
    double cd = distance_3d(C,D);
    double ad = distance_3d(D,A);        
    double bc = distance_3d(B,C);
    double bd = distance_3d(B,D);
    double ab = distance_3d(A,B);    
    

    // The lengths on b are AC, CD, DA.
    //    double b = heron_area(ac,cd,ad);
    //    double db_dx = darea_dx(cd,ac,ad);    
    
    // The lengths on a are BC,CD,BD.
    //    double a = heron_area(bc,cd,bd);
    //    double da_dx = darea_dx(cd,bc,bd);

    //    double c = heron_area(ab,ad,bd);
    //    double d = heron_area(ab,ac,bc);

    // sadly, I have been confused.  I think I need to use this,
    // which will require the cosine law to find vertex angles, from
    // which the dihedrals can be computed from the article.

    // https://math.stackexchange.com/questions/314970/dihedral-angles-between-tetrahedron-faces-from-triangles-angles-at-the-tip
    //    cout << "lengths ab, ac, ad, bc, bd " << ab << " " << ac << " " << ad << " " << bc << " " << bd << "\n";

    // if these are greater than 90 degrees, they are probably wrong!
    // The MAX_PHYSICAL_ANGLE depends on the min and max physically actualizable
    // lengths. WARNING: these outines are return 90 degress more than my angles,
    // I think!
    //    const MAX_PHYSICAL_ANGLE = 
    double vc = angle_from_three_sides(ad,ab,bd);
    double vd = angle_from_three_sides(ac,ab,bc);
    double vb = angle_from_three_sides(ac,ad,cd);

    //    cout << "vc, vd, vb " << vc*180.0/M_PI << " " << vd*180.0/M_PI << " " << vb*180.0/M_PI << "\n";

// find the dihedral angle of <AB given vertex angles...
//    double ztheta = dihedral_from_vertex_angles(vc,vd,vb);

    // now I need to compute change in theta via another method...
    double dvertex_angle_dlength = dangle_from_dside(ac,ad,cd);
    double ddihedral_dv = ddhedral_dvertex(vd,vc,vb);

    //    cout << "dva " << dvertex_angle_dlength << "\n";
    //    cout << "ddihedral_dv " << ddihedral_dv << "\n";

        double dvtheta = dvertex_angle_dlength * ddihedral_dv ;
    
    //    cout << "dvtheta " << dvtheta*180.0/M_PI << "\n";
    // possibly this should be negative, since it is an outside angle.
	double intnl_dvtheta_tenth = -dvtheta*0.1;

	//    cout << "dvtheta int " << intnl_dvtheta_tenth*180.0/M_PI << "\n";    
	//    cout << "theta differential " << 0.1*ytheta*180.0/M_PI << "\n";
    BOOST_CHECK(abs((xtheta+dtheta_tenth) - ytheta) < epsilon);
    BOOST_CHECK(abs((0.1*ytheta) - intnl_dvtheta_tenth) < epsilon);    

  }
  

  BOOST_CHECK(true);
}


// This tests our ability to solve the "forward" problem
// Note we will use right hand coordinates.
BOOST_AUTO_TEST_CASE( test_find_point_from_transformed )
{
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);

  // For testing, let's put:
  // A on the x axis,
  // B on the y axis
  // C on the z axis,
  // and then let's aim to have D be the unit point.
  A = 0.0,0.0,0.0;
  B = 1.0,0.0,0.0;
  C = 1.0,1.0,0.0;
  D = 1.0,1.0,1.0;

  // now compute the distances...
  double ab = distance_3d(A,B);
  cout << "XXXX\n";
  cout << ab;
  print_vec(A);
  print_vec(B);
  double ac = distance_3d(A,C);
  double ad = distance_3d(A,D);
  
  double bc = distance_3d(B,C);  
  double bd = distance_3d(B,D);
  double cd = distance_3d(C,D);

  cout << "distances \n";
  cout << ab << " ";
  cout << ac << " ";
  cout << ad << " ";
  cout << bc << " ";
  cout << bd << " ";
  cout << cd << " \n";
  

  column_vector Dprime = find_point_from_transformed(ab,ac,ad,bc,bd,cd);
  print_vec(D);
  print_vec(Dprime);
  BOOST_CHECK(equal(Dprime,D));
}


// This tests our ability to solve the "forward" problem
// Note we will use right hand coordinates.
BOOST_AUTO_TEST_CASE( test_transform_triangle )
{
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  // First let's test with a triangle in the xz plane
  A = 0.0,2.0,0.0;
  B = 1.0,2.0,1.0;
  C = 2.0,2.0,0.0;

  column_vector O(3);
  O = 0.0,0.0,0.0;

  point_transform_affine3d tform = compute_transform_to_axes2(A,B,C);

  column_vector Ap(3);
  column_vector Bp(3);
  column_vector Cp(3);

  Ap = tform(A);
  cout << "Ap \n";
  print_vec(Ap);
  
  BOOST_CHECK(equal(Ap,O));
  Bp = tform(B);
  cout << "Bp \n";
  print_vec(Bp);
  
  
  Cp = tform(C);
  cout << "Cp \n";
  print_vec(Cp);

  cout << tform.get_m() << "\n";
  cout << tform.get_b() << "\n";  
}

// BOOST_AUTO_TEST_CASE( test_inverse_transform )
// {
//   column_vector A(3);
//   A = 1.0,1.0,1.0;
//   double theta = M_PI/6;
//   point_transform_affine3d rotate0 = rotate_around_x(theta);
//   cout << "XXXXXXX\n";
//   cout << A << "\n";
//   cout << rotate0(A) << "\n";
//   cout << rotate0(inv(rotate0)(A)) << "\n";
//   cout << "XXXXXXX\n";  
  
// }

BOOST_AUTO_TEST_CASE( test_find_fourth_point )
{
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);

  // For testing, let's put:
  // A on the x axis,
  // B on the y axis
  // C on the z axis,
  // and then let's aim to have D be the unit point.
  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.1;
  C = 0.0,10.0,0.1;
  D = 5,5,5;

  // now compute the distances...
  double da = distance_3d(A,D);
  double db = distance_3d(B,D);
  double dc = distance_3d(C,D);
  

  // Now having done this, we should be able to get
  // D back out of system (subject to getting the normal right!)


  column_vector Dp = find_fourth_point_given_three_points_and_three_distances(CCW,
									     A,B,C,
									     da,db,dc);
  cout << "Dp = " << "\n";
  print_vec(Dp);
  column_vector diff = Dp - D;
  BOOST_CHECK(l2_norm(diff) < 1e-3);
}


// This tests our ability to solve the "forward" problem
BOOST_AUTO_TEST_CASE( find_coords )
{
  Tetrahelix thlx(20,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );

  cout << "START\n";
  cout << "thlx.num_nodes: " << thlx.num_nodes << "\n";
  column_vector* coords = new column_vector[thlx.num_nodes];

  //  const double SHORT = LOWER_BOUND;
  const double LONG = UPPER_BOUND;

  thlx.distance(0) = INITIAL;
  thlx.distance(1) = LONG;
  thlx.distance(2) = INITIAL;
  thlx.distance(3) = INITIAL;
  thlx.distance(4) = LONG;

  solve_forward_find_coords(&thlx,coords);

  for(int i = 0; i < thlx.num_nodes; i++) {
    cout << "coordsv " << i+1 << " :\n";
    print_vec(coords[i]);
  }
  // for(int i = 0; i < thlx.num_edges; i++) {
  //   cout << "distance  " << i << " ";
  //   cout << thlx.distance(i) << "\n";
  // }

  // cout << "num_edges " << thlx.num_edges << "\n";
  // for(int i = 4; i < thlx.num_edges; i++) {
  //       int ai = thlx.small_node(i);
  //       int bi = thlx.large_node(i);

    
  // 	cout << "distance  ai,bi :" << ai << "," << bi << " " << thlx.edge_between(bi,ai) << "\n";
  // 	cout << "coords[ai], coords[bi] :" << coords[ai] << "," << coords[bi] << "\n";
  // 	cout << distance_3d(coords[ai],coords[bi]) << "\n";
  // }

  // BOOST_CHECK(true);
}
