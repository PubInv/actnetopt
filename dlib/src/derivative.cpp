// derivative.cpp -- Test code for gradient-based optimization of 2d VGT manipulator
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


//Using SDL and standard IO
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include "TriLadder.hpp"
#include "Invert.hpp"
#include "Obstacle.hpp"

#include <math.h>

#define PI 3.14159265

#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>

#define TRUSS_NODES 10
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5


BOOST_AUTO_TEST_CASE( test_angle )
{
  TriLadder tl(TRUSS_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );

  column_vector xaxis(2);
  xaxis(0) = 1.0;
  xaxis(1) = 0.0;

  column_vector origin(2);
  origin(0) = 0.0;
  origin(1) = 0.0;

  column_vector yaxis(2);
  yaxis(0) = 0.0;
  yaxis(1) = 1.0;

  column_vector d(2);
  d(0) = 1.0;
  d(1) = 1.0;
  
  double theta = get_angle(xaxis, origin, d);
  cout << "theta = " << theta*180/PI << "\n";
  double epsilon = 0.0001;
  double target = -45.0;
  BOOST_CHECK(abs((theta * 180 /PI) - target) < epsilon);

  BOOST_CHECK(true);
}


BOOST_AUTO_TEST_CASE( test_sense )
{
  column_vector A(2);
  column_vector B(2);
  column_vector C(2);


  A(0) = 0.0;
  A(1) = 0.0;
  B(0) = 0.0;
  B(1) = 4.0;
  C(0) = 2.0;
  C(1) = 2.0;
  double s_angle_abc0 = -sense(A(0),A(1),B(0),B(1),C(0),C(1));
  //  double s_angle_abc0x = -sense2(A(0),A(1),B(0),B(1),C(0),C(1));
  cout << "s_angle_abc0 : " << s_angle_abc0 << "\n";
  BOOST_CHECK( s_angle_abc0 >  0.0 );
  //  BOOST_CHECK( s_angle_abc0 = s_angle_abc0x);  
  

  A(0) = 0.0;
  A(1) = 0.0;
  B(0) = 1.0;
  B(1) = 1.5;
  C(0) = 1.3;
  C(1) = 0.75;

  double s_angle_abc1 = -sense(A(0),A(1),B(0),B(1),C(0),C(1));
  cout << "s_angle_abc1 : " << s_angle_abc1 << "\n";
  BOOST_CHECK( s_angle_abc1 >  0.0 );

}

BOOST_AUTO_TEST_CASE( find_coords )
{
  TriLadder tl(TRUSS_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );

  cout << "START\n";
  cout << "tl.num_nodes: " << tl.num_nodes << "\n";
  column_vector* coords = new column_vector[tl.num_nodes];

  const double SHORT = LOWER_BOUND;
  const double LONG = UPPER_BOUND;

  tl.distance(0) = INITIAL;
  tl.distance(1) = LONG;
  tl.distance(2) = SHORT;
  tl.distance(3) = SHORT;
  tl.distance(4) = LONG;
  
  find_all_coords(&tl,coords);

  for(int i = 0; i < tl.num_nodes; i++) {
    cout << "coordsv " << i+1 << " :\n";
    print_vec(coords[i]);
  }
  for(int i = 0; i < tl.num_edges; i++) {
    cout << "distance  " << i << " ";
    cout << tl.distance(i) << "\n";
  }

  for(int i = 0; i < tl.num_edges; i++) {
    int ai = tl.small_node(i);
    int bi = tl.large_node(i);
    cout << "distance  ai,bi :" << ai << "," << bi << " " << tl.edge_between(bi,ai) << " ";
    cout << distance_2d(coords[ai],coords[bi]) << "\n";
  }

  BOOST_CHECK(true);
}

void print_vector_with_indices(column_vector d) {
  for(int i = 0; i < d.size(); i++) {
    cout << "i " << i << " " << d(i) << "\n";
  }
}

TriLadder *Invert::global_truss = 0;

BOOST_AUTO_TEST_CASE( two_node_derivative_behaves_accurately )
{
  TriLadder an(TRUSS_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );

  int last_node = an.num_nodes - 1;
  double bx = ((last_node % 2) == 0) ? 0.0 : an.median_d * cos(30.0*PI/180);
  double by = (an.median_d/2.0) * last_node;
  
  double mx = 1.0;
  double my = 1.0;
  an.add_goal_node(an.num_nodes-1,bx+mx,by+my,1.0);    

  for(int i = 0; i < an.goal_nodes.size(); i++) {
    cout << "goal_nodes[" << i << "] " << an.goal_nodes[i] << "\n";
  }


  cout << "START\n";
  cout << "an.num_nodes: " << an.num_nodes << "\n";
  column_vector* coords = new column_vector[an.num_nodes];

  const double SHORT = LOWER_BOUND;
  const double LONG = UPPER_BOUND;

  an.distance(0) = INITIAL;
  an.distance(1) = LONG;
  an.distance(2) = SHORT;
  an.distance(3) = SHORT;
  an.distance(4) = LONG;
  
  find_all_coords(&an,coords);

  Invert inv;
  inv.an = &an;
  // We'll put an obstacle far away since we aren't ready to deal with that here yet.
  Obstacle obstacle;
  obstacle.radius = 0.0;
  obstacle.center = column_vector(2);
  obstacle.center(0) = 1000.0;
  obstacle.center(1) = 1000.0;

  inv.set_global_truss(obstacle);
  
  cout << "==================\n";
  
  column_vector ds(an.var_edges);
  for (int i = 0; i < an.var_edges; ++i) {
      if (debug) std::cout << i << " : " << ds(i) << std::endl;
      // This is correct?  It should it just be "i"?
      cout << i << " " << an.distance(i+1) << "\n";
      ds(i) = an.distance(i+1);
  }
  cout << "==================\n";
  column_vector derivatives0 = inv.derivative(ds);

  cout << "DERIVATIVES" << "\n";
  for(int i = 0; i < an.var_edges; i++) {
    cout << "i " << i << " " << derivatives0(i) << "\n";
  }

  cout << "SECOND NODE ADDED\n";
  
  an.add_goal_node(an.num_nodes/2,bx/2,by/2,1.0);
  
  column_vector derivatives1 = inv.derivative(ds);
  
  for(int i = 0; i < an.var_edges; i++) {
    cout << "i " << i << " " << derivatives1(i) << "\n";
  }
    
  BOOST_CHECK(true);
}

BOOST_AUTO_TEST_CASE( Objective_function_appears_sensible )
{
  TriLadder an(TRUSS_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );

  int last_node = an.num_nodes - 1;
  double bx = ((last_node % 2) == 0) ? 0.0 : an.median_d * cos(30.0*PI/180);
  double by = (an.median_d/2.0) * last_node;
  
  double mx = 10.0;
  double my = 10.0;
  an.add_goal_node(an.num_nodes-1,bx+mx,by+my,1.0);    
  for(int i = 0; i < an.goal_nodes.size(); i++) {
    cout << "goal_nodes[" << i << "] " << an.goal_nodes[i] << "\n";
  }
  column_vector* coords = new column_vector[an.num_nodes];
  
  find_all_coords(&an,coords);

  Invert inv;
  inv.an = &an;
  Obstacle obstacle;
  obstacle.radius = 0.0;
  obstacle.center = column_vector(2);
  obstacle.center(0) = 1000.0;
  obstacle.center(1) = 1000.0;
  inv.set_global_truss(obstacle);
  
  column_vector ds(an.var_edges);
  for (int i = 0; i < an.var_edges; ++i) {
      if (debug) std::cout << i << " : " << ds(i) << std::endl;
      cout << i << " " << an.distance(i+1) << "\n";
      ds(i) = an.distance(i+1);
  }
  
  cout << "==================\n";
  double score0 = inv.objective(ds);
  cout << "score: " << score0 << "\n";

  an.add_goal_node(an.num_nodes/2,bx+mx,by+my,1.0);    

  cout << "==================\n";
  double score1 = inv.objective(ds);
  cout << "score: " << score1 << "\n";
  
  BOOST_CHECK(score1 > score0);
}

BOOST_AUTO_TEST_CASE( Create_an_obstacle )
{
  Obstacle o;
  o.radius = 1.0;
  o.center = column_vector(2);
  o.center(0) = 0.0;
  o.center(1) = 4.0;
  for(int i = 0; i < 5; i++ ) {
    column_vector x(2);
    x(0) = 0.0;
    x(1) = i*1.0;
    cout << i << "f " << o.f(x) << "\n";
  }
  for(int i = 0; i < 5; i++ ) {
    cout << i << "p " << o.partial(i*1.0) << "\n";
  }
}
