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
#include "playground3d.hpp"
#include "Invert3d.hpp"
#include "Obstacle.hpp"

using namespace dlib;
using namespace std;


#undef TRUSS_NODES
#define TRUSS_NODES 4


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
BOOST_AUTO_TEST_CASE( test_computation_change_angle_wrt_length )
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
  // Test: This needs to be tested with lower levels of theta.
  // I need to turn this into a routine to test with different thetas.
  // I also need a diagram; I can't remember what theta represents here.
  double theta = 60.0 * M_PI/ 180.0;
  double Cx = cos(theta);
  double Cy = sin(theta);
  A = 0.0,0.0,1.0;
  B = 0.0,0.0,-2.0;
  C = Cx,Cy,0.0;
  D = 1.0,0.0,0.0;

  // now we have a particular tetrahedron.
  // We want to compute d theta/ dl, where L = len(DC).
  double epsilon = 3.00 * M_PI/ 180.0;

  // The point of this test is to change the X value of D
  // computing at each instance and to comare the differential
  // between to values with the computed derivative.
  for(int i = 0; i < 5; i++) {

    // This is used to compute the differential.
    D(0) = (1.0 + i*0.1);
    double dist = distance_3d(C,D);

    // dtheta 
    double dtheta = dtheta_dd_laid_on_axes(dist,D(0),D(2));
    double dtheta_tenth = 0.1 * dtheta;
    cout << i << " " << dist << " " << dtheta*180.0/M_PI << "\n";
    //    cout << i << " " << dist << " " << dtheta_tenth*180.0/M_PI << "\n";

    //    double ctheta = atan2(C(1),C(0));
    //    cout << "ctheta " << ctheta*180.0/M_PI << "\n";
    // These are the angles as measured from the "forward" problem...
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


    // I think dvtheta is the external strap angle....
    double dvtheta = dvertex_angle_dlength * ddihedral_dv ;

    //    cout << "dvtheta " << dvtheta*180.0/M_PI << "\n";
    // possibly this should be negative, since it is an outside angle.
    double intnl_dvtheta_tenth = dvtheta*0.1;

    //    cout << "dvtheta int " << intnl_dvtheta_tenth*180.0/M_PI << "\n";    
    //    cout << "theta differential " << 0.1*ytheta*180.0/M_PI << "\n";

    // This is the differential....    
    BOOST_CHECK(abs((xtheta+dtheta_tenth) - ytheta) < epsilon);

    // This is the derivative...
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
  double ac = distance_3d(A,C);
  double ad = distance_3d(A,D);
  
  double bc = distance_3d(B,C);  
  double bd = distance_3d(B,D);
  double cd = distance_3d(C,D);

  

  bool valid;
  column_vector Dprime = find_point_from_transformed(CCW,ab,ac,ad,bc,bd,cd,&valid);
  //  print_vec(D);
  //  print_vec(Dprime);
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
  //  cout << "Ap \n";
  //  print_vec(Ap);
  // A should go to the origin....
  BOOST_CHECK(equal(Ap,O));

  // B should go to the x axis...
  Bp = tform(B);

  BOOST_CHECK(Bp(0) > 0);
  double epsilon = 1e-7;
  BOOST_CHECK(abs(Bp(1) - 0) < epsilon);
  BOOST_CHECK(abs(Bp(1) - 0) < epsilon);


  //  cout << "Bp \n";
  //  print_vec(Bp);
  
  
  Cp = tform(C);
  //  cout << "Cp \n";
  //  print_vec(Cp);

  //  cout << tform.get_m() << "\n";
  //  cout << tform.get_b() << "\n";  
}

BOOST_AUTO_TEST_CASE( test_transform_to_axes2_0 )
{
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);

  // For testing, let's put:
  A = 10.0,0.0,0.0;
  B = 0.0,10.0,0.0;
  C = 0.0,0.0,10.0;
  
 // First compute all 6 distances....
  //  double bc = distance_3d(B,C);
  //  double bd = distance_3d(B,D);
  //  double cd = distance_3d(D,D);
  
  
  // Now find transformation that rotates and translates to axes...
  int debug = 0;
  if (debug) cout << "A B C \n";
  if (debug) cout << A  << " " << B  << " " << C << "\n";  
  point_transform_affine3d tform = compute_transform_to_axes2(A,B,C);

  column_vector Cp(3);

  Cp = tform(C);
  if (debug) {
    print_vec(Cp);
    cout << "\n";
  }
  
  // Now, the point Cp needs to be in the XY plane (that is, Z = 0).
  BOOST_CHECK(abs(Cp(2)) < 1e-4);
}

BOOST_AUTO_TEST_CASE( test_transform_to_axes2_1 )
{
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);

  // For testing, let's put:
  A = -13.0,0.0,0.0;
  B = 4.0,10.0,0.0;
  C = 0.0,1.0,10.0;
  
 // First compute all 6 distances....
  //  double bc = distance_3d(B,C);
  //  double bd = distance_3d(B,D);
  //  double cd = distance_3d(D,D);
  
  
  // Now find transformation that rotates and translates to axes...
  int debug = 0;
  if (debug) cout << "A B C \n";
  if (debug) cout << A  << " " << B  << " " << C << "\n";  
  point_transform_affine3d tform = compute_transform_to_axes2(A,B,C);

  column_vector Cp(3);

  Cp = tform(C);
  if (debug) {
    print_vec(Cp);
    cout << "\n";
  }

  
  // Now, the point Ep needs to be in the XY plane (that is, Z = 0).
  BOOST_CHECK(abs(Cp(2)) < 1e-4);
}

BOOST_AUTO_TEST_CASE( test_transform_to_axes2_2 )
{
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);

  // For testing, let's put:
  A = 0.0,0.0,0.0;
  B = 0.0,10.0,0.0;
  C = 4.0,1.0,10.0;
  
 // First compute all 6 distances....
  //  double bc = distance_3d(B,C);
  //  double bd = distance_3d(B,D);
  //  double cd = distance_3d(D,D);
  
  
  // Now find transformation that rotates and translates to axes...
  int debug = 0;
  if (debug) cout << "A B C \n";
  if (debug) cout << A  << " " << B  << " " << C << "\n";  
  point_transform_affine3d tform = compute_transform_to_axes2(A,B,C);

  column_vector Cp(3);

  Cp = tform(C);
  if (debug) {
    print_vec(Cp);
    cout << "\n";
  }

  
  // Now, the point Ep needs to be in the XY plane (that is, Z = 0).
  BOOST_CHECK(abs(Cp(2)) < 1e-4);
}


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
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  D = 5,5,5;

  // now compute the distances...
  double da = distance_3d(A,D);
  double db = distance_3d(B,D);
  double dc = distance_3d(C,D);
  

  // Now having done this, we should be able to get
  // D back out of system (subject to getting the normal right!)


  bool valid;
  column_vector Dp = find_fourth_point_given_three_points_and_three_distances(CCW,
									      A,B,C,
									      da,db,dc,&valid);
  Chirality senseABCD = tet_chirality(A,B,C,D);
  Chirality senseABCDp = tet_chirality(A,B,C,Dp);

  int debug = 0;
  
  if (debug) {
    cout << "Dp = " << "\n";
    print_vec(Dp);
    cout << " chirality actual, computed " << senseABCD << " " << senseABCDp << "\n";
  }
  column_vector diff = Dp - D;
  BOOST_CHECK(senseABCD == senseABCDp);  
  BOOST_CHECK(l2_norm(diff) < 1e-3);
}


// This tests our ability to solve the "forward" problem
BOOST_AUTO_TEST_CASE( find_coords_matches_distances )
{
  Tetrahelix thlx(20,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL
		  );

  //  cout << "START\n";
  //  cout << "thlx.num_nodes: " << thlx.num_nodes << "\n";
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  TetrahelixConfiguration thc(&thlx,coords);

  //  const double SHORT = LOWER_BOUND;
  //  const double LONG = UPPER_BOUND;
  for(int i = 0; i < thlx.num_nodes; i++) {
    thlx.distance(i) = INITIAL;
  }

  thc.forward_find_coords();
  
  for(int i = 0; i < thlx.num_nodes; i++) {
    thlx.distance(i) = INITIAL;
    //    cout << i << " " << coords[i] << "\n";
  }

  //  cout << "num_edges " << thlx.num_edges << "\n";
  for(int i = 4; i < thlx.num_edges; i++) {
    int ai = thlx.small_node(i);
    int bi = thlx.large_node(i);
    double actual = distance_3d(coords[ai],coords[bi]);
    double expected = thlx.distance(i);
    BOOST_CHECK(abs(expected-actual) < 1e-2);
  }
  delete[] coords;
}


BOOST_AUTO_TEST_CASE( make_sure_axis_is_followed )
{
  Tetrahelix thlx(20,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....

  
  
  column_vector* coords = new column_vector[thlx.num_nodes];

  //  const double SHORT = LOWER_BOUND;
  //  const double LONG = UPPER_BOUND;
  for(int i = 0; i < thlx.num_nodes; i++) {
    thlx.distance(i) = INITIAL;
  }
  
  TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();

  
  for(int i = 0; i < thlx.num_nodes; i++) {
    thlx.distance(i) = INITIAL;
    //    cout << i << " " << coords[i] << "\n";
  }

  //  cout << "num_edges " << thlx.num_edges << "\n";
  for(int i = 4; i < thlx.num_nodes; i++) {
    //        int ai = thlx.small_node(i);
    //        int bi = thlx.large_node(i);
    //	double actual = distance_3d(coords[ai],coords[bi]);
    //	double expected = thlx.distance(i);

    // we want to make sure each point is not too far
    // from the y axis.
    // This is a little tricky...let me just test the coords
    // stay close to the x axis...

    //    cout << i << " " << coords[i](0) << "\n";
    BOOST_CHECK(abs(coords[i](0)) < 2.0);
  }
  delete[] coords;
}

BOOST_AUTO_TEST_CASE( test_compute_transform_to_axes2_invertable )
{
  column_vector A(3);
  column_vector B(3);
  column_vector Y(3);
  column_vector Q(3);  

  // For testing, let's put:
  // A on the orign,
  // B on the y axis
  // C on the z axis,
  // and then let's aim to have D be the unit point.
  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  Y = 0.0,10.0,0.0;
  Q = 10.0,10.0,10.0;  

  point_transform_affine3d tform = compute_transform_to_axes2(B,A,Y);
  //  cout << tform.get_m() << "\n";
  //  cout << tform.get_b() << "\n";  

  // This appears to be transforming the z axis....
  column_vector Yt = tform(Y);
  //  cout << "Should lie on the y axis\n";
  //  print_vec(Yt);

  double epsilon = 1e-6;
  BOOST_CHECK(abs(Yt(0) - 10.0) < epsilon);
  BOOST_CHECK(abs(Yt(1) + 10.0) < epsilon);
  BOOST_CHECK(abs(Yt(2) - 0.0) < epsilon);

  // This appears to be transforming the z axis....
  column_vector At = tform(A);
  //  cout << "Should lie on the y axis\n";
  print_vec(At);
  BOOST_CHECK(abs(At(0) - 10.0) < epsilon);
  BOOST_CHECK(abs(At(1) + 0.0) < epsilon);
  BOOST_CHECK(abs(At(2) - 0.0) < epsilon);

  column_vector Bt = tform(B);
  //  print_vec(Bt);
  BOOST_CHECK(abs(Bt(0) - 0.0) < epsilon);
  BOOST_CHECK(abs(Bt(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(Bt(2) - 0.0) < epsilon);

  column_vector Qt = tform(Q);
  print_vec(Qt);
  BOOST_CHECK(abs(Qt(0) - 0.0) < epsilon);
  BOOST_CHECK(abs(Qt(1) + 10.0) < epsilon);
  BOOST_CHECK(abs(Qt(2) - 10.0) < epsilon);

}

BOOST_AUTO_TEST_CASE( test_compute_transform_to_axes2 )
{
  column_vector A(3);
  column_vector B(3);
  column_vector Y(3);
  column_vector Q(3);  

  // For testing, let's put:
  // A on the orign,
  // B on the y axis
  // C on the z axis,
  // and then let's aim to have D be the unit point.
  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  Y = 0.0,10.0,0.0;
  Q = 10.0,10.0,10.0;  

  point_transform_affine3d tform = compute_transform_to_axes2(A,B,Y);

  // This appears to be transforming the z axis....
  column_vector Yt = tform(Y);
  //  cout << "Should lie on the y axis\n";
  //  print_vec(Yt);

  double epsilon = 1e-6;
  BOOST_CHECK(abs(Yt(0) - 0.0) < epsilon);
  BOOST_CHECK(abs(Yt(1) - 10.0) < epsilon);
  BOOST_CHECK(abs(Yt(2) - 0.0) < epsilon);

  // This appears to be transforming the z axis....
  column_vector At = tform(A);
  //  cout << "Should lie on the y axis\n";
  //  print_vec(At);
  BOOST_CHECK(abs(At(0) - 0.0) < epsilon);
  BOOST_CHECK(abs(At(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(At(2) - 0.0) < epsilon);

  column_vector Bt = tform(B);
  //  print_vec(Bt);
  BOOST_CHECK(abs(Bt(0) - 10.0) < epsilon);
  BOOST_CHECK(abs(Bt(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(Bt(2) - 0.0) < epsilon);

  column_vector Qt = tform(Q);
  //  print_vec(Qt);
  BOOST_CHECK(abs(Qt(0) - 10.0) < epsilon);
  BOOST_CHECK(abs(Qt(1) - 10.0) < epsilon);
  BOOST_CHECK(abs(Qt(2) - 10.0) < epsilon);

  // Now let's make B point out in all positive directions.

  A = 0.0,0.0,0.0;
  B = 10.0,10.0,10.0;
  Y = 0.0,100.0,0.0;
  Q = 100.0,0.0,0.0;

  point_transform_affine3d tform1 = compute_transform_to_axes2(A,B,Y);

  // This appears to be transforming the z axis....
  column_vector Yt1 = tform1(Y);
  //  print_vec(Yt1);

  // This appears to be transforming the z axis....
  column_vector At1 = tform1(A);
  //  cout << "At1\n";  
  //  print_vec(At1);
  BOOST_CHECK(abs(At1(0) - 0.0) < epsilon);
  BOOST_CHECK(abs(At1(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(At1(2) - 0.0) < epsilon);

  column_vector Bt1 = tform1(B);
  //  cout << "Bt1\n";
  //  print_vec(Bt1);
  BOOST_CHECK(abs(Bt1(0) - 17.3205) < 0.01);
  BOOST_CHECK(abs(Bt1(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(Bt1(2) - 0.0) < epsilon);

  column_vector Qt1 = tform1(Q);
  //  cout << "t1\n";    
  //  print_vec(Qt1);
  BOOST_CHECK(Bt1(0) > 0.0);
  BOOST_CHECK(Bt1(1) < 0.0);
  BOOST_CHECK(Bt1(2) < 0.0);
  
}

// This tests our ability to solve the "forward" problem
BOOST_AUTO_TEST_CASE( test_rotation_about_points )
{
  column_vector A(3);
  column_vector B(3);
  column_vector M(3);

  // For testing, let's put:
  // A on the orign,
  // B on the y axis
  // C on the z axis,
  // and then let's aim to have D be the unit point.
  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  M = 0.0,10.0,0.0;

  // first off, designing a case that will put M on the Z axis (X and Y = 0.0);
  double theta = M_PI/2;
  column_vector Mp;
  double epsilon = 1.0e-10;  
  
  Mp = compute_rotation_about_points(A,B,theta,M);
  cout << "Mp  A B\n";
  print_vec(Mp);

  BOOST_CHECK(abs(Mp(0) -  0.0) < epsilon);
  BOOST_CHECK(abs(Mp(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(Mp(2) - 10.0) < epsilon);

  Mp = compute_rotation_about_points(B,A,theta,M);

  cout << "Mp B A\n";
  print_vec(Mp);
  
  BOOST_CHECK(abs(Mp(0) -  0.0) < epsilon);
  BOOST_CHECK(abs(Mp(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(Mp(2) + 10.0) < epsilon);
  
}
// This tests our ability to solve the "forward" problem
BOOST_AUTO_TEST_CASE( test_rotation_about_points_sense )
{
  column_vector A(3);
  column_vector B(3);
  column_vector M(3);

  // For testing, let's put:
  // A on the orign,
  // B on the y axis
  // C on the z axis,
  // and then let's aim to have D be the unit point.
  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  M = 10.0,10.0,0.0;

  // first off, designing a case that will put M on the Z axis (X and Y = 0.0);
  // This is 45 degrees.
  double theta = M_PI/4;
  column_vector Mp_pos;
  column_vector Mp_neg;  
  double epsilon = 1.0e-10;
  double factor = sin(theta);

  // This is an anti-clockwise rotation about the vector A to B...
  Mp_pos = compute_rotation_about_points(A,B,theta,M);

  // Z should be positive, y positive, X unchange
  BOOST_CHECK(abs(Mp_pos(0) -  10.0) < epsilon);
  BOOST_CHECK(abs(Mp_pos(1) - 10.0*factor) < epsilon);
  BOOST_CHECK(abs(Mp_pos(2) - 10.0*factor) < epsilon);

  Mp_neg = compute_rotation_about_points(A,B,-theta,M);

  BOOST_CHECK(abs(Mp_neg(0) -  10.0) < epsilon);
  BOOST_CHECK(abs(Mp_neg(1) - 10.0*factor) < epsilon);
  BOOST_CHECK(abs(Mp_neg(2) + 10.0*factor) < epsilon);
  BOOST_CHECK(abs(Mp_neg(2) + Mp_pos(2)) < epsilon);  

}

BOOST_AUTO_TEST_CASE( test_can_compute_chirality )
{
  Tetrahelix thlx(4,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....

  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);  

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = 0.0,0.0,0.0;
  B = 1.0,0.0,0.0;
  C = 0.0,1.0,0.0;
  // Note if that D has a negaitive Z axis, is the D is not anti-clockwise relative to BC.
  // We should disallow clockwise tetrahedra; in our system, you are not allowed to turn
  // a tetrahedron inside-out in our physical machine.
  // We should write a test to make sure that we can distinguish such.
  D = 0.0,0.0,-10.0;

  // first, let's test that we can compute opposite normals...

  column_vector N = normal(A,B,C);
  BOOST_CHECK(N(0) == 0);
  BOOST_CHECK(N(1) == 0);
  BOOST_CHECK(N(2) == 1.0);    
}

BOOST_AUTO_TEST_CASE( test_compute_goal_derivative_c )
{
  Tetrahelix thlx(4,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....

  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);  

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  // Note if that D has a negaitive Z axis, is the D is not anti-clockwise relative to BC.
  // We should disallow clockwise tetrahedra; in our system, you are not allowed to turn
  // a tetrahedron inside-out in our physical machine.
  // We should write a test to make sure that we can distinguish such.
  D = 0.0,0.0,-10.0;      
  
  Chirality senseABC = tet_chirality(A,B,C,D);
  Chirality senseACB = tet_chirality(A,C,B,D);  
  BOOST_CHECK(senseABC == CW);
  BOOST_CHECK(senseACB == CCW);

  // comput_goal_derivative_c should fail if we manage to give it a CW tetrahedron!
}

int debug = 0;
// Tetrahelix *Invert3d::global_truss = 0;

BOOST_AUTO_TEST_CASE( test_distance_to_goal0 )
{
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix thlx(4,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];

  //  const double SHORT = LOWER_BOUND;
  //  const double LONG = UPPER_BOUND;
  for(int i = 0; i < thlx.num_edges; i++) {
    thlx.distance(i) = INITIAL;
  }

  TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();


  int debug = 0;
  if (debug) {
    cout << "DONE WITH FORWARD\n";

    // Just check this is what I expect...
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }
  
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  if (debug) {
    cout << "BBB\n";
  }

  thlx.add_goal_node(3,0.0,0.0,-20.0,1.0);

  if (debug) {
    // now check that we have one goal...
    for(int i = 0; i < thlx.goal_nodes.size(); i++) {
      cout << "goal_nodes " << i << " " << thlx.goal_nodes[i] << "\n";
    }
    cout << "CCC\n";
  }
  
  column_vector gl(3);
  gl(0) = 0.0;
  gl(1) = 2.5;
  gl(2) = 5.0;
  thlx.goals[thlx.goals.size() - 1] = gl;

  if (debug) {
    // Now our goal should be set.
    for(int i = 0; i < thlx.goal_nodes.size(); i++) {
      cout << "goal_nodes " << i << "  " << thlx.goal_nodes[i] << "\n";
    }
  }
  
  column_vector ds(thlx.var_edges);

  const column_vector* dsa = &ds;
  
  for(int i = 0; i < thlx.var_edges; i++) {
    ds(i) = UPPER_BOUND;
  }
  for(int i = 0; i < thlx.var_edges; i++) {
    thlx.distance(i+3) = ds(i);
  }

  thc.forward_find_coords();


  column_vector deriv0 = inv.derivative(*dsa);

  if (debug) {
    cout << "DERIVATIVE COMPUTED\n";
    for (int i = 0; i < thlx.var_edges; ++i) {
      int n = thlx.edge_number_of_nth_variable_edge(i);
      cout << "edge " << n << " " << deriv0(i) << "\n";
    }
  }
  
    thc.forward_find_coords();
  
  
  double v0 = inv.objective(*dsa);
  
  for(int i = 0; i < thlx.var_edges; i++) {
    ds(i) = LOWER_BOUND;
  }
  for(int i = 0; i < thlx.var_edges; i++) {
    thlx.distance(i+3) = ds(i);
  }
  
  thc.forward_find_coords();

  
  // cout << "Coords at MIN\n";
  // for (int i = 0; i < thlx.num_nodes; ++i) {
  //   cout << " i : ";
  //   print_vec(coords[i]);
  // }

  // cout << " distances \n";
  // for (int i = 0; i < thlx.num_nodes-1; ++i) {
  //   for (int j = i+1; j <  thlx.num_nodes; ++j) {    
  //     cout << " i,j : " << i << "," << j << " ";
  //     cout << distance_3d(coords[i],coords[j]) << "\n";
  //   }
  // }

  double v1 = inv.objective(*dsa);
  // object at max should be lower than the objective at min!!!!
  BOOST_CHECK( v0 < v1 );

  delete[] coords;
}


// compute the derivative of the tetrahelix against the goal for testing
column_vector compute_derivative_for_single_tet_testing(Invert3d *inv,Tetrahelix *an,
							column_vector* coords,
							column_vector goal,double *obj) {

  // First set all distances based on coords.
  column_vector A = coords[0];
  column_vector B = coords[1];
  column_vector C = coords[2];
  column_vector D = coords[3];

  an->set_distances(coords);  

  column_vector ds(an->var_edges);
  const column_vector* dsa = &ds;

  for(int i = 0; i < an->var_edges; i++) {
    int n = an->edge_number_of_nth_variable_edge(i);    
    ds(i) = an->distance(n);
  }

  an->goals[an->goals.size() - 1] = goal;

  column_vector deriv =  inv->derivative(*dsa);
  
  *obj = inv->objective(*dsa);
  
  return deriv;
}

BOOST_AUTO_TEST_CASE( test_directional_derivatives )
{
  //  cout << "TEST DERIVATIVES WITH AXES COORDS\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix thlx(4,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector Dgoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  D = 3.0,3.0,11.0;
  // Dgoal is not actually used here...
  Dgoal = 0.0,0.0,12.0;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;


  thlx.set_distances(coords);

  // Now we will lengthen edge 2 by precisely 0.1...
  //  thlx.distance(2) += 1.0;

  // This is really 4 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();


  column_vector goal = Dgoal;
  thlx.add_goal_node(3,Dgoal(0),Dgoal(1),Dgoal(2),1.0);

  column_vector d2 = thlx.compute_goal_derivative_c(coords,2,thlx.goal_nodes[0]);
  //  cout << "computed directional derivative for edge 2:\n";
  //  print_vec(d2);
  // These are the directions for changing edge 2.
  // X should go down. Y should go up. Z should go up slightly.
  BOOST_CHECK( d2(0) > 0 );
  BOOST_CHECK( d2(1) > 0 );
  BOOST_CHECK( d2(2) > 0 );

  column_vector d4 = thlx.compute_goal_derivative_c(coords,4,thlx.goal_nodes[0]);
  //  cout << "computed directional derivative for edge 4:\n";
  //  print_vec(d4);
  // These are the directions for changing edge 4.
  // X should go down. Y should go up. Z should go up slightly.
  BOOST_CHECK( d4(0) < 0 );
  BOOST_CHECK( d4(1) == 0 );
  BOOST_CHECK( d4(2) > 0 );

  column_vector d5 = thlx.compute_goal_derivative_c(coords,5,thlx.goal_nodes[0]);
  //  cout << "computed directional derivative for edge 5:\n";
  //  print_vec(d5);
  // These are the directions for changing edge 4.
  // X should go down. Y should go up. Z should go up slightly.
  BOOST_CHECK( d5(0) == 0 );
  BOOST_CHECK( d5(1) < 0 );
  BOOST_CHECK( d5(2) > 0 );
  
  delete[] coords;  
}

// This is an attempt to set up a very particular situation which allows
// us to test effectively.
BOOST_AUTO_TEST_CASE( test_derivatives_by_making_sure_length_increases )
{
  //  cout << "TEST DERIVATIVES WITH AXES COORDS\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix thlx(4,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector Dgoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  D = 0.0,0.0,10.0;
  Dgoal = 0.0,0.0,15.0;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;

  thlx.set_distances(coords);

  // Now we will lengthen edge 2 by precisely 0.1...
  //  thlx.distance(2) += 1.0;

  // This is really 4 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);
  
  // cout << "ABOUT TO SOLVE_FORWARD_FIND_COORDS \n";
  // for(int i = 0; i < thlx.num_nodes; i++) {
  //   print_vec(coords[i]);
  // }
  
  TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();


  column_vector goal = Dgoal;

  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  // Our goal now in this test is to place the goal node
  // in a direct relationship specific distances.
  // This should then allow us to compute a nearly "pure" derivative.
  // For example, we length edge 2 by 0.1, and compute this goal position.
  // Presumably then the derivative (the change of edge length towards the goal)
  // should be positive for edge 2 but zero for others.
  thlx.add_goal_node(3,Dgoal(0),Dgoal(1),Dgoal(2),1.0);
  
  thc.forward_find_coords();
  
  double obj;
  column_vector deriv = compute_derivative_for_single_tet_testing(&inv,inv.an,
								  coords,goal,&obj);
  
  thlx.distance(2) = 15.0;
  
  thc.forward_find_coords();

  deriv = compute_derivative_for_single_tet_testing(&inv,inv.an,
						    coords,goal,&obj);
  
  BOOST_CHECK( deriv(0) > 0 );
  BOOST_CHECK( deriv(1) < 0 );
  BOOST_CHECK( deriv(2) < 0 );
  delete[] coords;
}

// This is an attempt to set up a very particular situation which allows
// us to test effectively.
BOOST_AUTO_TEST_CASE( test_derivatives )
{
  //  cout << "TEST DERIVATIVES WITH AXES COORDS\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix thlx(4,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector Dgoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  D = 0.0,0.0,10.0;
  Dgoal = 0.0,0.0,12.0;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;

  thlx.set_distances(coords);

  // Now we will lengthen edge 2 by precisely 0.1...
  //  thlx.distance(2) += 1.0;

  // This is really 4 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);
  
  // cout << "ABOUT TO SOLVE_FORWARD_FIND_COORDS \n";
  // for(int i = 0; i < thlx.num_nodes; i++) {
  //   print_vec(coords[i]);
  // }
  
  TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();


  column_vector goal = Dgoal;

  // cout << "Goal node based on modified edge length:";

  // // This is wrong, and it fails!
  // print_vec(coords[3]);

  // now restore the system...
  //  thlx.distance(2) -= 1.0;
  
  // cout << "initial position of node #3:";
  // print_vec(coords[3]);
  

  // // Just check this is what I expect...
  // for(int i = 0; i < thlx.num_nodes; i++) {
  //   print_vec(coords[i]);
  // }
  
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  //  cout << "BBB\n";  

  // Our goal now in this test is to place the goal node
  // in a direct relationship specific distances.
  // This should then allow us to compute a nearly "pure" derivative.
  // For example, we length edge 2 by 0.1, and compute this goal position.
  // Presumably then the derivative (the change of edge length towards the goal)
  // should be positive for edge 2 but zero for others.
  thlx.add_goal_node(3,Dgoal(0),Dgoal(1),Dgoal(2),1.0);
  
  // cout << "number of goals: " << thlx.goals.size() << "\n";
  
  // print_vec(thlx.goals[0]);

  double obj;
  column_vector deriv = compute_derivative_for_single_tet_testing(&inv,inv.an,
								  coords,goal,&obj);
  cout << "Objective: " << obj << "\n";

  // column_vector ds(thlx.var_edges);

  // const column_vector* dsa = &ds;

  // for(int i = 0; i < thlx.var_edges; i++) {
  //   int n = thlx.edge_number_of_nth_variable_edge(i);    
  //   ds(i) = thlx.distance(n);
  // }


  // // set up the goal correctly....
  // column_vector deriv = inv.derivative(*dsa);

  int debug = 0;
  for(int i = 0; i < thlx.var_edges; i++) {
    int n = thlx.edge_number_of_nth_variable_edge(i);    
    if (debug) cout << "Deriv " << n << " " << deriv(i) << "\n";
  }
  BOOST_CHECK( deriv(0) < 0 );
  BOOST_CHECK( deriv(1) > 0 );
  BOOST_CHECK( deriv(2) > 0 );

  // TODO: Compute objective here to make sure it is right.

  if (debug) cout << "TESTING DERIVATIVE AGAINST Z\n";
  for(int j = 0; j < 10; j++) {
    goal(2) = 10.0+(5-j)*1.0;
    column_vector deriv = compute_derivative_for_single_tet_testing(&inv,inv.an,
								    coords,goal,&obj);
    if (debug) cout << "Obj, Deriv: " << obj << " ";    
    if (debug) print_vec(goal);
    for(int i = 0; i < thlx.var_edges; i++) {
      int n = thlx.edge_number_of_nth_variable_edge(i);    
      if (debug) cout << n << " " << deriv(i) << "   ";
    }
    if (debug) cout << "\n";
  }
  
  goal(2) = 10.0;

  if (debug) cout << "TESTING DERIVATIVE AGAINST Y\n";  
  for(int j = 0; j < 10; j++) {
    goal(1) = 2.0+(5-j)*1.0;
    column_vector deriv = compute_derivative_for_single_tet_testing(&inv,inv.an,
								    coords,goal,&obj);
    if (debug) print_vec(goal);
    if (debug) cout << "Obj, Deriv: " << obj << " ";    
    for(int i = 0; i < thlx.var_edges; i++) {
      int n = thlx.edge_number_of_nth_variable_edge(i);    
      if (debug) cout << n << " " << deriv(i) << "   ";
    }
    if (debug) cout << "\n";
  }

  goal(1) = 0.0;
  
  if (debug) cout << "TESTING DERIVATIVE AGAINST X\n";  
  for(int j = 0; j < 10; j++) {
    goal(0) = 5.0+(5-j)*1.0;
    column_vector deriv = compute_derivative_for_single_tet_testing(&inv,inv.an,
								    coords,goal,&obj);
    if (debug) cout << "Obj, Deriv: " << obj << " ";
    if (debug) print_vec(goal);
    for(int i = 0; i < thlx.var_edges; i++) {
      int n = thlx.edge_number_of_nth_variable_edge(i);    
      if (debug) cout << n << " " << deriv(i) << "   ";
    }
    if (debug) cout << "\n";
  }
  delete[] coords;
}

BOOST_AUTO_TEST_CASE( test_solving_when_goal_is_current_position)
{
  //  cout << "TEST SOLVING WHEN GOAL IS CURRENT TETRAHEDRONXSD\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix thlx(4,
		  20.0,
		  10.0,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector Dgoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  D = 1.0,0.0,11.0;
  Dgoal = 1.0,0.0,11.0;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;


  thlx.set_distances(coords);
  
  // Now we will lengthen edge 2 by precisely 0.1...
  //  thlx.distance(2) += 1.0;

  // This is really 4 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();


  //   cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
  if (debug) {
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }

  column_vector goal = Dgoal;
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  const int goal_node = 3;
  thlx.add_goal_node(goal_node,Dgoal(0),Dgoal(1),Dgoal(2),1.0);

  // Now we will create an iteration of points in space, checking many of them.
  thlx.goals[thlx.goals.size() - 1] = goal;
  double obj;
  column_vector deriv = compute_derivative_for_single_tet_testing(&inv, inv.an, coords, goal, &obj);
  if (debug) {
    cout << "DERIV FOR GOAL = CUR\n";
    print_vec(deriv);
    cout << "DONE WITH SOLVING\n";  
  }
  BOOST_CHECK(deriv(0) == 0);
  BOOST_CHECK(deriv(1) == 0);
  BOOST_CHECK(deriv(2) == 0);    
  delete[] coords;
}

// This really only works with a SINGLE tetrahedron; I just check the first fixed coords against the
// goal --- this does not work in any more general situation, so I am putting this here in the test file.
int errant_max_distance(double lower_bound, double upper_bound, column_vector coords[],int len,column_vector goal) {
  int debug = 0;
  for(int i = 0; i < len - 1 ; i++) {
    double d = distance_3d(coords[i],goal);
    if (d < lower_bound) {
      if (debug) {
	cout << "ERRANT LOW: " << d << "\n";
	print_vec(coords[i]);
	print_vec(goal);
      }
      return i;
    } if (d > upper_bound) {
      if (debug) {
	cout << "ERRANT HIGH: " << d << "\n";
	print_vec(coords[i]);
	print_vec(goal);
      }
      return i;
    }
  }
  return -1;
}


// This is an attempt to set up a very particular situation which allows
// us to test effectively.
BOOST_AUTO_TEST_CASE( test_ability_to_solve_a_single_tetrahedron )
{
  //  cout << "TEST ABILITY TO SOLVE A SINGLE TETRAHEDRONXSD\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix thlx(4,
		  18.0,
		  10.0,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector Dgoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  D = 0.0,0.0,10.0;
  Dgoal = 0.0,0.0,11.0;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;

  thlx.set_distances(coords);

  // Now we will lengthen edge 2 by precisely 0.1...
  //  thlx.distance(2) += 1.0;

  // This is really 4 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();


  if (debug) {
    cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }

  column_vector goal = Dgoal;
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  const int goal_node = 3;
  thlx.add_goal_node(goal_node,Dgoal(0),Dgoal(1),Dgoal(2),1.0);

  // Now we will create an iteration of points in space, checking many of them.

  int debug = 0;
  for(int i = 0; i < 8; i++) {
    goal(0) = i * 1.0;
    for(int j = 0; j < 8; j++ ) {
      goal(1) = j * 1.0;
      thlx.goals[thlx.goals.size() - 1] = goal;
      
      if (debug) cout << "Experiment (i,j) :" << i << " " << j <<  "\n";
	
      int e = errant_max_distance(thlx.lower_bound,thlx.upper_bound,coords,4,goal);
      if (e > -1) {
	cout << "errant edge: " << e << "\n";
	print_vec(goal);
      } else {
	double obj;
	if (debug) cout << "pre-computation\n";
      
	if (debug) print_vec(coords[goal_node]);
      
	column_vector deriv = compute_derivative_for_single_tet_testing(&inv, inv.an, coords, goal, &obj);
	if (debug) {
	  cout << "deriv \n";
	  print_vec(deriv);
	  cout << "post-computation\n";
	  print_vec(coords[goal_node]);
	}
	if (isnan(deriv(0))) { // This means that the current coords is actually equal to the goal node
	  cout << "deriv is undefined!\n";
	} else {
	  //	  cout << "deriv:\n";
	  //	  print_vec(deriv);
	  //	  for (int i = 0; i < thlx.num_edges; ++i) {
	  //	    cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
	  //	  }
	  solve_inverse_problem(inv.an);
	  // cout << "solving forward!\n";
	  bool solved =   thc.forward_find_coords();
	  if (!solved) {
	    cout << "INTERNAL ERROR: Could solved these coords forward!\n";
	  }
	}
	if (debug) {
	  cout << "coords!!!\n";
	  print_vec(coords[goal_node]);
	  print_vec(goal);
	  cout << "NORM: " << l2_norm(coords[goal_node] - goal) << "\n";	  
	};


	BOOST_CHECK(l2_norm(coords[goal_node] - goal) < 1e-2);
      }
    }
  }

  // Just check this is what I expect...
  for(int i = 0; i < thlx.num_nodes; i++) {
    print_vec(coords[i]);
  }
  delete[] coords;
}


// // This is an attempt to set up a very particular situation which allows
// // us to test effectively.
BOOST_AUTO_TEST_CASE( test_ability_to_solve_a_double_tetrahedron )
{
  //  cout << "TEST ABILITY TO SOLVE A SINGLE TETRAHEDRONXSD\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix thlx(5,
		  18.0,
		  10.0,
		  MEDIAN,
		  INITIAL
		  );

  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector E(3);
  column_vector Egoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  D = 0.0,0.0,10.0;
  // put the fourth node at a cube corner!
  E = 10.0,10.0,10.0;
  // Let's extend the E node just a little at first...
  Egoal = 12.0,12.0,12.0;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;
  coords[4] = E;  



  // This is really 5 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  thlx.set_distances(coords);  
  int debug = 0;
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();

  cout << " BCDE chirality :" << tet_chirality(B,C,D,E) << "\n";

  if (debug) {
    cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }
  
  const int goal_node = 4;
  
  // THE point E should be close to the cube corner
  double d = l2_norm(coords[goal_node] - E);
  cout << "CHECK DISTANCE (SMALL)" << d << "\n";
  BOOST_CHECK(l2_norm(coords[goal_node] - E) < 1e-2);
  
  //  column_vector goal = Egoal;
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();



  thlx.add_goal_node(goal_node,Egoal(0),Egoal(1),Egoal(2),1.0);

  // Now we will create an iteration of points in space, checking many of them.
  int num = 4;
  for(int i = 0; i < num; i++) {
    Egoal(0) = 10.0 +  i * 1.0;
    for(int j = 0; j < num; j++ ) {
      Egoal(1) = 10.0 + j * 1.0;
      thlx.goals[thlx.goals.size() - 1] = Egoal;
      
      if (debug) cout << "Experiment (i,j) :" << i << " " << j <<  "\n";
	
      // int e = errant_max_distance(thlx.lower_bound,thlx.upper_bound,coords,4,Egoal);
      // if (e > -1) {
      // 	cout << "errant edge length: " << e << "\n";
      // 	print_vec(Egoal);
      // } else {
  	double obj;
  	if (debug) cout << "pre-computation\n";
      
  	if (debug) print_vec(coords[goal_node]);
      
  	column_vector deriv = compute_derivative_for_single_tet_testing(&inv, inv.an, coords, Egoal, &obj);
  	if (debug) {
  	  cout << "deriv \n";
  	  print_vec(deriv);
  	  cout << "post-computation\n";
  	  print_vec(coords[goal_node]);
  	}
  	if (isnan(deriv(0))) { // This means that the current coords is actually equal to the goal node
  	  cout << "deriv is undefined!\n";
  	} else {
  	  //	  cout << "deriv:\n";
  	  //	  print_vec(deriv);
  	  //	  for (int i = 0; i < thlx.num_edges; ++i) {
  	  //	    cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
  	  //	  }
  	  solve_inverse_problem(inv.an);
  	  // cout << "solving forward!\n";
  	  bool solved =   thc.forward_find_coords();
  	  if (!solved) {
  	    cout << "INTERNAL ERROR: Could solved these coords forward!\n";
  	  }
  	}
  	if (debug) {
  	  cout << "coords!!!\n";
  	  print_vec(coords[goal_node]);
  	  print_vec(Egoal);
  	  cout << "NORM: " << l2_norm(coords[goal_node] - Egoal) << "\n";	  
  	};

  	BOOST_CHECK(l2_norm(coords[goal_node] - Egoal) < 1e-2);
	//      }
    }
  }

  // // Just check this is what I expect...
  // for(int i = 0; i < thlx.num_nodes; i++) {
  //   print_vec(coords[i]);
  // }
  delete[] coords;
}

BOOST_AUTO_TEST_CASE( test_rail_dege_p) {
  Tetrahelix *thlx_ptr = init_Tetrahelix(5,2.0,1.2,1.5);
  Tetrahelix thlx = *thlx_ptr;
  for(int i = 0; i < thlx.num_edges; i++) {
    //    bool rail = thlx.rail_edge_p(i);
    // cout << "edge, rail: " << i << " " << rail << "\n";
    // cout << "edge, simple: " << i << " " << thlx.simple_hinge_p(i) << "\n";    
  }
}

// This is to test a double tetrahedron in smaller coordinates
// which I have had some trouble with in the "playground" GUI that I am trying.
BOOST_AUTO_TEST_CASE( test_internal_edge_d_dihedral_computation_double_tetrahedron_with_playground_coords )
{
  cout << "TEST INTERNAL EDGE d_DIHEDRAL DOUBLE TETRAHEDRON \n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix *thlx_ptr = init_Tetrahelix(5,2.0,1.2,1.5);
  Tetrahelix thlx = *thlx_ptr;
  
  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector E(3);
  //  column_vector Egoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = thlx.fixed[0];
  B = thlx.fixed[1];
  C = thlx.fixed[2];
  // This is the "standard" solution.
  D = 0.6350852961085883,2.790116647275517,-1.57697505292423;

  double Ex = -0.7601778544330073;
  double Ey = 2.5104011833827586;
  double Ez = -1.102633403898972;
  
  E = Ex,Ey,Ez;

  // Let's extend the E node just a little at first...
  //  Egoal = Ex,Ey+0.1,Ez;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;
  coords[4] = E;  

  // This is really 5 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  thlx.set_distances(coords);  
  int debug = 0;
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }

  //  thlx.goals[thlx.goals.size() - 1] = Egoal;
      
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();

  
  if (debug) {
    cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }
  
  for(int i = 0; i < thlx.var_edges; i++) {
    int n = thlx.edge_number_of_nth_variable_edge(i);
    bool internal = !thlx.simple_hinge_p(n);
    if (internal) {
      cout << "internal edge: " << n << "\n";
      double d_dihedralBC = thlx.d_dihedralBC_dCD(coords,n);
      cout << d_dihedralBC*180.0/M_PI << "\n";
    } 
  }
}

BOOST_AUTO_TEST_CASE( test_internal_edge_derivative_double_tetrahedron_with_playground_coords )
{
  cout << "TEST INTERNAL EDGE DERIVATIVE DOUBLE TETRAHEDRON \n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix *thlx_ptr = init_Tetrahelix(5,2.0,1.2,1.5);
  Tetrahelix thlx = *thlx_ptr;
  
  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector E(3);
  //  column_vector Egoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = thlx.fixed[0];
  B = thlx.fixed[1];
  C = thlx.fixed[2];
  // This is the "standard" solution.
  D = 0.6350852961085883,2.790116647275517,-1.57697505292423;

  double Ex = -0.7601778544330073;
  double Ey = 2.5104011833827586;
  double Ez = -1.102633403898972;
  
  E = Ex,Ey,Ez;

  // Let's extend the E node just a little at first...
  //  Egoal = Ex,Ey+0.1,Ez;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;
  coords[4] = E;  

  // This is really 5 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  thlx.set_distances(coords);  
  int debug = 0;
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }

  //  thlx.goals[thlx.goals.size() - 1] = Egoal;
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();

  
  if (debug) {
    cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }
  // Note, for edges 4 and 5 these should both be positive.
  // I am compute B and C wrong in that function.
  for(int i = 0; i < thlx.var_edges; i++) {
    int n = thlx.edge_number_of_nth_variable_edge(i);
    bool internal = !thlx.simple_hinge_p(n);
    if (internal) {
      cout << "derivative coming up internal edge: " << n << "\n";
      column_vector d_derivBC = thlx.compute_goal_derivative_after_edge_internal(coords,n,4);
      print_vec(d_derivBC);
      // I'm not sure how to turn this into a good test --- I suppose
      // cmputing a differential is the only way.
    } 
  }
}


// This is to test a double tetrahedron in smaller coordinates
// which I have had some trouble with in the "playground" GUI that I am trying.
BOOST_AUTO_TEST_CASE( test_ability_to_solve_unique_double_tetrahedron_with_playground_coords )
{
  cout << "TEST ABILITY TO SOLVE A UNIQUE DOUBLE TETRAHEDRON \n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix *thlx_ptr = init_Tetrahelix(5,2.0,1.2,1.5);
  Tetrahelix thlx = *thlx_ptr;
  
  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector E(3);
  column_vector Egoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = thlx.fixed[0];
  B = thlx.fixed[1];
  C = thlx.fixed[2];
  // This is the "standard" solution.
  D = 0.6350852961085883,2.790116647275517,-1.57697505292423;

  double Ex = -0.7601778544330073;
  double Ey = 2.5104011833827586;
  double Ez = -1.102633403898972;
  
  E = Ex,Ey,Ez;

  // Let's extend the E node just a little at first...
  Egoal = Ex,Ey+0.1,Ez;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;
  coords[4] = E;  

  // This is really 5 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  thlx.set_distances(coords);  
  int debug = 0;
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();

  cout << " BCDE chirality :" << tet_chirality(B,C,D,E) << "\n";

  if (debug) {
    cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }
  
  const int goal_node = 4;
  
  // THE point E should be close to the cube corner
  double d = l2_norm(coords[goal_node] - E);
  cout << "CHECK DISTANCE (SMALL)" << d << "\n";
  BOOST_CHECK(l2_norm(coords[goal_node] - E) < 1e-2);
  
  //  column_vector goal = Egoal;
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  // Note: using init creates a goal node already!
  thlx.goals[thlx.goals.size() - 1] = Egoal;  
  solve_inverse_problem(inv.an);

  bool solved =   thc.forward_find_coords();
  if (!solved) {
    cout << "INTERNAL ERROR: Could solved these coords forward!\n";
  }
  if (debug) {
    cout << "coords!!!\n";
    print_vec(coords[goal_node]);
    print_vec(Egoal);
    cout << "NORM: " << l2_norm(coords[goal_node] - Egoal) << "\n";	  
  };
  BOOST_CHECK(l2_norm(coords[goal_node] - Egoal) < 1e-2);

}

// This is to test a double tetrahedron in smaller coordinates
// which I have had some trouble with in the "playground" GUI that I am trying.
BOOST_AUTO_TEST_CASE( test_ability_to_solve_a_double_tetrahedron_with_playground_coords )
{
  //  cout << "TEST ABILITY TO SOLVE A SINGLE TETRAHEDRONXSD\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix *thlx_ptr = init_Tetrahelix(5,2.0,1.2,1.5);
  Tetrahelix thlx = *thlx_ptr;
  
  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector E(3);
  column_vector Egoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = thlx.fixed[0];
  B = thlx.fixed[1];
  C = thlx.fixed[2];
  // This is the "standard" solution.
  D = 0.6350852961085883,2.790116647275517,-1.57697505292423;

  double Ex = -0.7601778544330073;
  double Ey = 2.5104011833827586;
  double Ez = -1.102633403898972;
  
  E = Ex,Ey,Ez;

  // Let's extend the E node just a little at first...
  Egoal = Ex,Ey+0.1,Ez;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;
  coords[4] = E;  
// This is really 5 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  thlx.set_distances(coords);  
  
  int debug = 0;
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();

  cout << " BCDE chirality :" << tet_chirality(B,C,D,E) << "\n";

  if (debug) {
    cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }
  
  const int goal_node = 4;
  
  // THE point E should be close to the cube corner
  double d = l2_norm(coords[goal_node] - E);
  cout << "CHECK DISTANCE (SMALL)" << d << "\n";
  BOOST_CHECK(l2_norm(coords[goal_node] - E) < 1e-2);
  
  //  column_vector goal = Egoal;
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  // Note: using init creates a goal node already!
  thlx.goals[thlx.goals.size() - 1] = Egoal;  

  // Now we will create an iteration of points in space, checking many of them.
  int numi = 4;
  int numj = 4;
  int numk = 4;
  double frac = 0.15;
  for(int i = 0; i < numi; i++) {
    Egoal(0) = Ex +  (numi/2 - i) * frac;
    for(int j = 0; j < numj; j++ ) {
      Egoal(1) = Ey + (numj/2 -j) * frac;
    for(int k = 0; k < numk; k++ ) {
      Egoal(2) = Ez + (numk/2 - k) * frac;
      thlx.goals[thlx.goals.size() - 1] = Egoal;
      
      if (debug) cout << "Experiment (i,j) :" << i << " " << j <<  "\n";
      if (debug) print_vec(Egoal);
      if (debug) cout << "BBB\n";
      if (debug) print_vec(thlx.goals[thlx.goals.size() - 1]); 
	
      //      int e = errant_max_distance(thlx.lower_bound,thlx.upper_bound,coords,4,Egoal);
      // if (e > -1) {
      // 	cout << "errant edge: " << e << "\n";
      // 	print_vec(Egoal);
      // } else {
      double obj;
      if (debug) cout << "pre-computation\n";
      
      if (debug) print_vec(coords[goal_node]);
      
      column_vector deriv = compute_derivative_for_single_tet_testing(&inv, inv.an, coords, Egoal, &obj);
      if (debug) {
	cout << "deriv \n";
	print_vec(deriv);
	cout << "post-computation\n";
	print_vec(coords[goal_node]);
      }
      if (isnan(deriv(0))) { // This means that the current coords is actually equal to the goal node
	cout << "deriv is undefined!\n";
      } else {
	if (debug) cout << "YYY " << thlx.goals.size() - 1 << "\n";
	if (debug) print_vec(inv.an->goals[thlx.goals.size() - 1]); 
	solve_inverse_problem(inv.an);
	
	if (debug) {
	  for (int i = 0; i < thlx.num_edges; ++i) {
	    cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
	  }
	}

	// cout << "solving forward!\n";
	bool solved =   thc.forward_find_coords();
	if (!solved) {
	  cout << "INTERNAL ERROR: Could solved these coords forward!\n";
	}
      }
      if (debug) {
	cout << "coords!!!\n";
	print_vec(coords[goal_node]);
	print_vec(Egoal);
	cout << "NORM: " << l2_norm(coords[goal_node] - Egoal) << "\n";	  
      };


      // This is not too precise but seems to be close, may have to track that down.
      BOOST_CHECK(l2_norm(coords[goal_node] - Egoal) < 1e-1);
    }
    }    
    //        }
  }

  // // Just check this is what I expect...
  // for(int i = 0; i < thlx.num_nodes; i++) {
  //   print_vec(coords[i]);
  // }
  delete[] coords;
}


// This is a new tack; I am attempting to copy the work of
// "Dynamic Simulation of Tetrahedron-Based Tetrobot" by Woo Ho Lee and Arthur C. Sanderson
// It purports to explain the computation of the Jacobian via linear algebra.
// I have been attempting this through pure geometry (and having a rough go of it.)

BOOST_AUTO_TEST_CASE( test_WooHoLee_Jacobian_Computation )
{
  //  cout << "TEST ABILITY TO SOLVE A SINGLE TETRAHEDRONXSD\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix *thlx_ptr = init_Tetrahelix(4,20.0,10.0,10.0);
  Tetrahelix thlx = *thlx_ptr;
  
  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  // This is the "standard" solution.
  D = 0.0, 0.0, 10.0;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;

// This is really 4 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  thlx.set_distances(coords);  
  
  int debug = 0;
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();


  // My goal here is to compute the Jacobian as per Lee-Sanderson and
  // compare ti to the differential.
  // Here I shall attempt to compute it---not even sure what shape it is!
  matrix<double> J_3 = thlx.JacobianBase(coords);

 for(int i = 0; i < thlx.var_edges; i++) {
    column_vector edge_length(thlx.var_edges);
    edge_length = 0.0,0.0,0.0 ;
    edge_length(i) = 1.0;
    matrix<double> deriv_i = normalize(J_3*edge_length);
    cout << "i, deriv:\n";
    cout << deriv_i;
    cout << "\n";
    
    int n = thlx.edge_number_of_nth_variable_edge(i);
    column_vector diff = normalize(thlx.compute_goal_differential_c(coords,n,3));
    cout << "differential:\n";
    print_vec(diff);
    column_vector deriv_col(3);
    deriv_col = deriv_i(0),deriv_i(1),deriv_i(2);
    double expect_vs_actual = l2_norm(diff-deriv_col);
    cout << "discrepancy:" << " " << expect_vs_actual << "\n";
    BOOST_CHECK(expect_vs_actual  < 5e-2);
    edge_length(i) = 0.0;    
  }
}

BOOST_AUTO_TEST_CASE( test_WooHoLee_Jacobian_Computation_second )
{
  //  cout << "TEST ABILITY TO SOLVE A SINGLE TETRAHEDRONXSD\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix *thlx_ptr = init_Tetrahelix(7,20.0,10.0,10.0);
  Tetrahelix thlx = *thlx_ptr;
  
  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector E(3);
  column_vector F(3);
  column_vector G(3);    

  A = 0.0,0.0,0.0;
  B = 10.0,0.0,0.0;
  C = 0.0,10.0,0.0;
  // This is the "standard" solution.
  D = 0.0, 0.0, 10.0;
  E = 10.0,10.0,10.0;
  F = 10.0,10.0,20.0;
  G = 10.0,20.0,20.0;    

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;
  coords[4] = E;
  coords[5] = F;
  coords[6] = G;      

// This is really 4 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);


  for (int i = 3; i < thlx.num_edges; ++i) {
    thlx.distance(i) = 15.0;
  }
  
  //  thlx.set_distances(coords);  
  
  int debug = 0;
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();


  // My goal here is to compute the Jacobian as per Lee-Sanderson and
  // compare ti to the differential.
  // Here I shall attempt to compute it---not even sure what shape it is!
  thlx.initialize_Jacobian_memo();
  thlx.fill_Jacobian(coords);
  matrix<double> Ju = thlx.get_Jacobian(coords,thlx.num_nodes-1);

  cout << "Jacobian:\n";
  cout << Ju;
  cout << "End Jacobian\n";

  thlx.Jacobian_temp = Ju;

  column_vector edge_length_deriv(thlx.var_edges);
  for(int i = 0; i < thlx.var_edges; i++) {
    edge_length_deriv(i) = 0.0;    
  }
  
  for(int i = 0; i < thlx.var_edges; i++) {
      edge_length_deriv(i) = 1.0;
      matrix<double> deriv_i = normalize(Ju*edge_length_deriv);
      cout << "i, deriv:\n";
      cout << i << " " << deriv_i;
      cout << "\n";

      int n = thlx.edge_number_of_nth_variable_edge(i);
      column_vector diff = normalize(thlx.compute_goal_differential_c(coords,n,thlx.num_nodes-1));

      cout << "differential:\n";
      print_vec(diff);
      
      column_vector deriv_col(3);
      deriv_col = deriv_i(0),deriv_i(1),deriv_i(2);
      double expect_vs_actual = l2_norm(diff-deriv_col);
      cout << "discrepancy:" << " " << expect_vs_actual << "\n";
      BOOST_CHECK(expect_vs_actual  < 5e-2);
      edge_length_deriv(i) = 0.0;      
  }
}


// This needs to be changed to test the Jacobian.
BOOST_AUTO_TEST_CASE( test_ability_to_solve_a_double_tetrahedron_with_playground_coords_Jacobian )
{
  //  cout << "TEST ABILITY TO SOLVE A SINGLE TETRAHEDRONXSD\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix *thlx_ptr = init_Tetrahelix(5,2.0,1.2,1.5);
  Tetrahelix thlx = *thlx_ptr;
  
  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector E(3);
  column_vector Egoal(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = thlx.fixed[0];
  B = thlx.fixed[1];
  C = thlx.fixed[2];
  // This is the "standard" solution.
  D = 0.6350852961085883,2.790116647275517,-1.57697505292423;

  double Ex = -0.7601778544330073;
  double Ey = 2.5104011833827586;
  double Ez = -1.102633403898972;
  
  E = Ex,Ey,Ez;

  // Let's extend the E node just a little at first...
  Egoal = Ex,Ey+0.1,Ez;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  coords[3] = D;
  coords[4] = E;  
// This is really 5 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  thlx.set_distances(coords);  
  
  int debug = 0;
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
    TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();

  cout << " BCDE chirality :" << tet_chirality(B,C,D,E) << "\n";

  if (debug) {
    cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }

  thlx.initialize_Jacobian_memo();
  thlx.fill_Jacobian(coords);
  matrix<double> Ju = thlx.get_Jacobian(coords,thlx.num_nodes-1);

  cout << "Jacobian:\n";
  cout << Ju;
  cout << "End Jacobian\n";

  thlx.Jacobian_temp = Ju;
  
  const int goal_node = 4;
  
  // THE point E should be close to the cube corner
  double d = l2_norm(coords[goal_node] - E);
  cout << "CHECK DISTANCE (SMALL)" << d << "\n";
  BOOST_CHECK(l2_norm(coords[goal_node] - E) < 1e-1);
  
  //  column_vector goal = Egoal;
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  // Note: using init creates a goal node already!
  thlx.goals[thlx.goals.size() - 1] = Egoal;  

  // Now we will create an iteration of points in space, checking many of them.
  int numi = 4;
  int numj = 4;
  int numk = 4;
  double frac = 0.15;
  for(int i = 0; i < numi; i++) {
    Egoal(0) = Ex +  (numi/2 - i) * frac;
    for(int j = 0; j < numj; j++ ) {
      Egoal(1) = Ey + (numj/2 -j) * frac;
    for(int k = 0; k < numk; k++ ) {
      Egoal(2) = Ez + (numk/2 - k) * frac;
      thlx.goals[thlx.goals.size() - 1] = Egoal;
      
      if (debug) cout << "Experiment (i,j) :" << i << " " << j <<  "\n";
      if (debug) print_vec(Egoal);
      if (debug) cout << "BBB\n";
      if (debug) print_vec(thlx.goals[thlx.goals.size() - 1]); 
	
      //      int e = errant_max_distance(thlx.lower_bound,thlx.upper_bound,coords,4,Egoal);
      // if (e > -1) {
      // 	cout << "errant edge: " << e << "\n";
      // 	print_vec(Egoal);
      // } else {
      double obj;
      if (debug) cout << "pre-computation\n";
      
      if (debug) print_vec(coords[goal_node]);
      
      column_vector deriv = compute_derivative_for_single_tet_testing(&inv, inv.an, coords, Egoal, &obj);
      // Here I compute the derivative from the stuff.
      if (debug) {
	cout << "deriv \n";
	print_vec(deriv);
	cout << "post-computation\n";
	print_vec(coords[goal_node]);
      }
      if (isnan(deriv(0))) { // This means that the current coords is actually equal to the goal node
	cout << "deriv is undefined!\n";
      } else {
	if (debug) cout << "YYY " << thlx.goals.size() - 1 << "\n";
	if (debug) print_vec(inv.an->goals[thlx.goals.size() - 1]); 
	solve_inverse_problem(inv.an);
	
	if (debug) {
	  for (int i = 0; i < thlx.num_edges; ++i) {
	    cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
	  }
	}

	// cout << "solving forward!\n";
	bool solved =   thc.forward_find_coords();
	if (!solved) {
	  cout << "INTERNAL ERROR: Could solved these coords forward!\n";
	}
      }
      if (debug) {
	cout << "coords!!!\n";
	print_vec(coords[goal_node]);
	print_vec(Egoal);
	cout << "NORM: " << l2_norm(coords[goal_node] - Egoal) << "\n";	  
      };

      BOOST_CHECK(l2_norm(coords[goal_node] - Egoal) < 1e-1);
    }
    }    
    //        }
  }

  // // Just check this is what I expect...
  // for(int i = 0; i < thlx.num_nodes; i++) {
  //   print_vec(coords[i]);
  // }
  delete[] coords;
}



// This needs to be changed to test the Jacobian.
BOOST_AUTO_TEST_CASE( test_ability_to_solve_a_large_tetrobot )
{
  //  cout << "TEST ABILITY TO SOLVE A SINGLE TETRAHEDRONXSD\n";
  // This is an attempt to make sure that the "distance_to_goal" after
  // solving our "standard start" tetrahelix goes down
  // if variable edges get bigger
  Tetrahelix *thlx_ptr = init_Tetrahelix(15,2.0,1.2,1.5);
  Tetrahelix thlx = *thlx_ptr;
  
  // Now we want to set up the coordinates of the first three nodes
  // very carefully so that we follow the X-axis specifically.
  // The easiest way to to this is to take it from the javascript
  // code already written to preform these calculations....
  column_vector* coords = new column_vector[thlx.num_nodes];
  
  column_vector A(3);
  column_vector B(3);
  column_vector C(3);
  column_vector D(3);
  column_vector E(3);
  column_vector G(3);    

  // Note we use right-handed coordinates.
  // This diagram matches (approximately) the diagram in the paper.

  A = thlx.fixed[0];
  B = thlx.fixed[1];
  C = thlx.fixed[2];
  // This is the "standard" solution.
  D = 0.6350852961085883,2.790116647275517,-1.57697505292423;

  double Gx = 0;
  double Gy = 0;
  double Gz = -3.0 + thlx.num_nodes*0.35;
  
  // Let's extend the E node just a little at first...
  G = Gx,Gy+0.1,Gz;

  // Note: This is done in this order so that we -Z will be 
  coords[0] = A;
  coords[1] = B;
  coords[2] = C;
  //  coords[3] = D;
  //  coords[4] = E;  
// This is really 5 points, but it will just read the first three..
  thlx.init_fixed_coords(coords);

  thlx.set_distances(coords);  
  
  int debug = 0;
  
  for (int i = 0; i < thlx.num_edges; ++i) {
    thlx.distance(i) = 1.5;
    //    cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
  }
  
  if (debug) {
    for (int i = 0; i < thlx.num_edges; ++i) {
      cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
    }
  }
  
  TetrahelixConfiguration thc(&thlx,coords);
  thc.forward_find_coords();

  cout << " BCDE chirality :" << tet_chirality(B,C,D,E) << "\n";

  if (debug) {
    cout << "DONE SOLVING_FORWARD_FIND_COORDS \n";
    for(int i = 0; i < thlx.num_nodes; i++) {
      print_vec(coords[i]);
    }
  }
  
  thlx.initialize_Jacobian_memo();
  thlx.fill_Jacobian(coords);
  
  matrix<double> Ju = thlx.get_Jacobian(coords,thlx.num_nodes-1);
  cout << "YYY\n";
  
  cout << "Jacobian:\n";
  cout << Ju;
  cout << "End Jacobian\n";

  thlx.Jacobian_temp = Ju;
  cout << "ZZZZ\n";
  
  const int goal_node = thlx.num_nodes-1;
  
  
  Invert3d inv;
  inv.an = &thlx;
  inv.set_global_truss();

  // Note: using init creates a goal node already!
  thlx.goals[thlx.goals.size() - 1] = G;  

  // Now we will create an iteration of points in space, checking many of them.
  int numi = 4;
  int numj = 4;
  int numk = 4;
  double frac = 0.15;
    cout << "ZZZZ\n";
    debug = 0;
  for(int i = 0; i < numi; i++) {
    G(0) = Gx +  (numi/2 - i) * frac;
    for(int j = 0; j < numj; j++ ) {
      G(1) = Gy + (numj/2 -j) * frac;
    for(int k = 0; k < numk; k++ ) {
      G(2) = Gz + (numk/2 - k) * frac;
      thlx.goals[thlx.goals.size() - 1] = G;
      
      if (debug) cout << "Dxperiment (i,j) :" << i << " " << j <<  "\n";
      if (debug) print_vec(G);
      if (debug) cout << "BBB\n";
      if (debug) print_vec(thlx.goals[thlx.goals.size() - 1]); 
	
      double obj;
      if (debug) cout << "pre-computation\n";
      
      if (debug) print_vec(coords[goal_node]);
      
      column_vector deriv = compute_derivative_for_single_tet_testing(&inv, inv.an, coords, G, &obj);
      // Here I compute the derivative from the stuff.
      if (debug) {
	cout << "deriv \n";
	print_vec(deriv);
	cout << "post-computation\n";
	print_vec(coords[goal_node]);
      }
      if (isnan(deriv(0))) { // This means that the current coords is actually equal to the goal node
	cout << "deriv is undefined!\n";
      } else {
	if (debug) cout << "YYY " << thlx.goals.size() - 1 << "\n";
	if (debug) print_vec(inv.an->goals[thlx.goals.size() - 1]); 
	solve_inverse_problem(inv.an);
	
	if (debug) {
	  for (int i = 0; i < thlx.num_edges; ++i) {
	    cout << " i, distances(i) " << i << " , " << thlx.distance(i) << "\n";
	  }
	}

	// cout << "solving forward!\n";
	bool solved =   thc.forward_find_coords();
	if (!solved) {
	  cout << "INTERNAL ERROR: Could solved these coords forward!\n";
	}
      }
      if (debug) {
	cout << "coords!!!\n";
	print_vec(coords[goal_node]);
	print_vec(G);
	cout << "NORM: " << l2_norm(coords[goal_node] - G) << "\n";	  
      };
      debug = 1;
      if (debug) {
	cout << "DISTANCE: " << l2_norm(coords[goal_node] - G) << "\n";	
      }
      debug = 0;
      
      if (l2_norm(coords[goal_node] - G) >= 1e-1) {
	cout << "DISTANCE: " << l2_norm(coords[goal_node] - G) << "\n";
	cout << " G = " << G << "\n";
	cout << " goal_node = " << coords[goal_node] << "\n";	
      }
      BOOST_CHECK(l2_norm(coords[goal_node] - G) < 1e-1);
    }
    }    
    //        }
  }

  // // Just check this is what I expect...
  // for(int i = 0; i < thlx.num_nodes; i++) {
  //   print_vec(coords[i]);
  // }
  delete[] coords;

    printf ("Time in Jacobian %lu clicks (%f seconds).\n",time_in_jacobian,((float)time_in_jacobian)/CLOCKS_PER_SEC);
    printf ("Time in Differential %lu clicks (%f seconds).\n",time_in_differential,((float)time_in_differential)/CLOCKS_PER_SEC);    
  
}
BOOST_AUTO_TEST_CASE( final_stats_report )
{
    printf ("Time in Jacobian %lu clicks (%f seconds).\n",time_in_jacobian,((float)time_in_jacobian)/CLOCKS_PER_SEC);
    printf ("Time in Differential %lu clicks (%f seconds).\n",time_in_differential,((float)time_in_differential)/CLOCKS_PER_SEC);    

}
