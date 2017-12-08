//Using SDL and standard IO
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include "TriLadder.h"


#include <math.h>

#define PI 3.14159265

#define BOOST_TEST_MODULE MyTest
#include <boost/test/unit_test.hpp>

#define LADDER_NODES 6
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5


BOOST_AUTO_TEST_CASE( test_angle )
{
  TriLadder tl(LADDER_NODES,
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


BOOST_AUTO_TEST_CASE( my_test )
{
  TriLadder tl(LADDER_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );
  cout << "object made\n";
  column_vector* coords = new column_vector[tl.num_nodes];  
  find_all_coords(&tl,coords);
  // for(int i = 0; i < tl.num_nodes*4; i++) {
  //   cout << i << " L " <<  tl.large_node(i) << "\n";
  // }
  // for(int i = 0; i < tl.num_nodes*4; i++) {
  //   cout << i << " S " <<  tl.small_node(i) << "\n";          
  // }

  
  // Edge 1: negative
  // Edge 2: positive
  for(int i = 0; i < tl.var_edges; i++) {
    double dtheta = tl.compute_single_derivative_dtheta(coords,i);
    cout << i+1 << " : " << dtheta << "\n";
  }
  //  std::cout << x << "," << y << "\n";

  BOOST_CHECK(true);
}

BOOST_AUTO_TEST_CASE( node_derivative )
{
  TriLadder tl(LADDER_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );
  cout << "START\n";
  column_vector* coords = new column_vector[tl.num_nodes];  
  find_all_coords(&tl,coords);

  for(int i = 0; i < tl.var_edges; i++) {
    column_vector d = tl.compute_single_derivative_c(coords,i);
    cout << "deriv " << i+1 << " :\n";
    print_vec(d);

  }
  //  std::cout << x << "," << y << "\n";

  BOOST_CHECK(true);
}


BOOST_AUTO_TEST_CASE( test_triangle_motion )
{
  // First, we define a triangle as three points
  column_vector a(2);
  column_vector b(2);
  column_vector c(2);
  a(0) = 0.0;
  a(1) = 0.0;

  b(0) = 3.0;
  b(1) = 3.0;

  c(0) = 3.0;
  c(1) = 0.0;


  // These are target points...
  column_vector ta(2);
  column_vector tb(2);
  ta(0) = 0.5;
  ta(1) = 0.0;
  tb(0) = 3.5;
  tb(1) = 3.0;

  double theta = PI*0.05;
  column_vector mp(2);
  mp(0) = 1.0;
  mp(1) = 1.0;
  ta = rotate_point<double>(mp,ta,theta);
  tb = rotate_point<double>(mp,tb,theta);    
    
  column_vector da(2);
  column_vector db(2);
  da = ta-a;
  db = tb-b;
  cout << "da/db\n";
  print_vec(da);
  print_vec(db);  
  
  // Then we send in the triangle and the motion of two points
  column_vector dc(2);
  dc = change_in_third_point(a,b,c,da,db);
  cout << "DC = " << dc << "\n";
  // We get back a new triangle and differential of the third point..
  // We check that the distances are all the same...
  // We check that points have been properly moved.
  column_vector ap(2);
  column_vector bp(2);
  column_vector cp(2);
  ap = a + da;
  bp = b + db;
  cp = c + dc;
  
  BOOST_CHECK_CLOSE( distance_2d(a,b), distance_2d(ap,bp),.0001 );
  BOOST_CHECK_CLOSE( distance_2d(b,c), distance_2d(bp,cp),.0001 );
  BOOST_CHECK_CLOSE( distance_2d(c,a), distance_2d(cp,ap),.0001 );  

}

