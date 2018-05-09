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

using namespace dlib;
using namespace std;


#define TRUSS_NODES 4
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5





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
    
    cout << "dvtheta " << dvtheta*180.0/M_PI << "\n";
    // possibly this should be negative, since it is an outside angle.
    double intnl_dvtheta_tenth = -dvtheta*0.1;

    cout << "dvtheta int " << intnl_dvtheta_tenth*180.0/M_PI << "\n";    
    cout << "theta differential " << 0.1*ytheta*180.0/M_PI << "\n";

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
  

  column_vector Dprime = find_point_from_transformed(CCW,ab,ac,ad,bc,bd,cd);
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
  // A should go to the origin....
  BOOST_CHECK(equal(Ap,O));

  // B should go to the x axis...
  Bp = tform(B);

  BOOST_CHECK(Bp(0) > 0);
  double epsilon = 1e-7;
  BOOST_CHECK(abs(Bp(1) - 0) < epsilon);
  BOOST_CHECK(abs(Bp(1) - 0) < epsilon);


  cout << "Bp \n";
  print_vec(Bp);
  
  
  Cp = tform(C);
  cout << "Cp \n";
  print_vec(Cp);

  cout << tform.get_m() << "\n";
  cout << tform.get_b() << "\n";  
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

  //  const double SHORT = LOWER_BOUND;
  //  const double LONG = UPPER_BOUND;
  for(int i = 0; i < thlx.num_nodes; i++) {
    thlx.distance(i) = INITIAL;
  }

  solve_forward_find_coords(&thlx,coords);
  
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

  solve_forward_find_coords(&thlx,coords);
  
  for(int i = 0; i < thlx.num_nodes; i++) {
    thlx.distance(i) = INITIAL;
    //    cout << i << " " << coords[i] << "\n";
  }

  cout << "num_edges " << thlx.num_edges << "\n";
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
  cout << "Should lie on the y axis\n";
  print_vec(Yt);

  double epsilon = 1e-6;
  BOOST_CHECK(abs(Yt(0) - 0.0) < epsilon);
  BOOST_CHECK(abs(Yt(1) - 10.0) < epsilon);
  BOOST_CHECK(abs(Yt(2) - 0.0) < epsilon);

  // This appears to be transforming the z axis....
  column_vector At = tform(A);
  cout << "Should lie on the y axis\n";
  print_vec(At);
  BOOST_CHECK(abs(At(0) - 0.0) < epsilon);
  BOOST_CHECK(abs(At(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(At(2) - 0.0) < epsilon);

  column_vector Bt = tform(B);
  print_vec(Bt);
  BOOST_CHECK(abs(Bt(0) - 10.0) < epsilon);
  BOOST_CHECK(abs(Bt(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(Bt(2) - 0.0) < epsilon);

  column_vector Qt = tform(Q);
  print_vec(Qt);
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
  print_vec(Yt1);

  // This appears to be transforming the z axis....
  column_vector At1 = tform1(A);
  cout << "At1\n";  
  print_vec(At1);
  BOOST_CHECK(abs(At1(0) - 0.0) < epsilon);
  BOOST_CHECK(abs(At1(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(At1(2) - 0.0) < epsilon);

  column_vector Bt1 = tform1(B);
  cout << "Bt1\n";
  print_vec(Bt1);
  BOOST_CHECK(abs(Bt1(0) - 17.3205) < 0.01);
  BOOST_CHECK(abs(Bt1(1) - 0.0) < epsilon);
  BOOST_CHECK(abs(Bt1(2) - 0.0) < epsilon);

  column_vector Qt1 = tform1(Q);
  cout << "t1\n";    
  print_vec(Qt1);
  BOOST_CHECK(Bt1(0) > 0.0);
  BOOST_CHECK(Bt1(1) < 0.0);
  BOOST_CHECK(Bt1(2) < 0.0);
  
}

// // This tests our ability to solve the "forward" problem
// BOOST_AUTO_TEST_CASE( test_rotation_about_points )
// {
//   column_vector A(3);
//   column_vector B(3);
//   column_vector M(3);

//   // For testing, let's put:
//   // A on the orign,
//   // B on the y axis
//   // C on the z axis,
//   // and then let's aim to have D be the unit point.
//   A = 0.0,0.0,0.0;
//   B = 10.0,0.0,0.0;
//   M = 0.0,10.0,0.0;

//   // first off, designing a case that will put M on the Z axis (X and Y = 0.0);
//   double theta = M_PI/2;
//   column_vector Mp;
//   double epsilon = 1.0e-10;  
  
//   Mp = compute_rotation_about_points(A,B,theta,M);
//   cout << "Mp  A B\n";
//   print_vec(Mp);

//   BOOST_CHECK(abs(Mp(0) -  0.0) < epsilon);
//   BOOST_CHECK(abs(Mp(1) - 0.0) < epsilon);
//   BOOST_CHECK(abs(Mp(2) - 10.0) < epsilon);

//   Mp = compute_rotation_about_points(B,A,theta,M);

//   cout << "Mp B A\n";
//   print_vec(Mp);
  
//   BOOST_CHECK(abs(Mp(0) -  0.0) < epsilon);
//   BOOST_CHECK(abs(Mp(1) - 0.0) < epsilon);
//   BOOST_CHECK(abs(Mp(2) + 10.0) < epsilon);
  
  
// }


// BOOST_AUTO_TEST_CASE( test_compute_goal_derivative_c )
// {
//   Tetrahelix thlx(4,
// 			   UPPER_BOUND,
// 			   LOWER_BOUND,
// 			   MEDIAN,
// 			   INITIAL
// 	       );

//   // Now we want to set up the coordinates of the first three nodes
//   // very carefully so that we follow the X-axis specifically.
//   // The easiest way to to this is to take it from the javascript
//   // code already written to preform these calculations....

//   column_vector A(3);
//   column_vector B(3);
//   column_vector C(3);
//   column_vector D(3);  

//   // Note we use right-handed coordinates.
//   // This diagram matches (approximately) the diagram in the paper.

//   A = 0.0,0.0,0.0;
//   B = 10.0,0.0,0.0;
//   C = 0.0,10.0,0.0;
//   D = 0.0,0.0,-10.0;      
  
//   column_vector* coords = new column_vector[thlx.num_nodes];

//   coords[0] = A;
//   coords[1] = B;
//   coords[2] = C;
//   coords[3] = D;

//   for(int i = 0; i < thlx.num_nodes; i++) {
//     thlx.distance(i) = INITIAL;
//   }

//   //  solve_forward_find_coords(&thlx,coords);

//   cout << "AAA\n";
  
//   column_vector gl(3);

//   thlx.add_goal_node(3,0.0,0.0,-20.0,1.0);
  
//   cout << thlx.goals.size() << "\n";
//   column_vector d;
//   d = thlx.compute_goal_derivative_c(coords,5,thlx.goal_nodes[0]);
//   cout << d << "\n";

//   // Basically the point is driven downward and a little on the positive x axis...
//   BOOST_CHECK(d(0) == 0);
//   BOOST_CHECK(d(1) < -1.4);
//   BOOST_CHECK(d(2) < 0.1);

//   // Now if we change edge 4, we expect to get: -X, +Y, -Z
//   d = thlx.compute_goal_derivative_c(coords,4,thlx.goal_nodes[0]);
//   cout << d << "\n";
//   BOOST_CHECK(d(0) < 0);
//   BOOST_CHECK(d(1) > 0);
//   BOOST_CHECK(d(2) < 0);
  
//   // We could check this better by changing the distance and running the solver,
//   // making sure that the endpoint is approximately in the place pointed to by the vector.
//   // If made into a callable routine this could be quite valuable.

//   // now if we change edge #2 instead, we expect z to move more negative,
  
//   // and to get slightly positive, and y to maybe get more positive.
//   // cout << "CCC\n";    
//   // d = thlx.compute_goal_derivative_c(coords,2,thlx.goal_nodes[0]);
//   // cout << d << "\n";

  
// }
