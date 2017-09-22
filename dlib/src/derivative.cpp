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

#define LADDER_NODES 10
#define UPPER_BOUND 2.0
#define LOWER_BOUND 1.2
#define MEDIAN 1.5
#define INITIAL 1.5


BOOST_AUTO_TEST_CASE( my_test )
{
  TriLadder tl(LADDER_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );
    double x;
    double y;
    tl.compute_single_derivative(tl.coords,2,&x,&y);
    std::cout << x << "," << y << "\n";

    BOOST_CHECK( x == 3 );
    BOOST_CHECK( y == 4 );
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
  
  BOOST_CHECK_CLOSE( distance(a,b), distance(ap,bp),.0001 );
  BOOST_CHECK_CLOSE( distance(b,c), distance(bp,cp),.0001 );
  BOOST_CHECK_CLOSE( distance(c,a), distance(cp,ap),.0001 );  

}

BOOST_AUTO_TEST_CASE( find_one_derivative )
{
  TriLadder tl(LADDER_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
			   );
    double x;
    double y;
    tl.compute_single_derivative(tl.coords,2,&x,&y);
    std::cout << x << "," << y << "\n";

    BOOST_CHECK( x == 3 );
    BOOST_CHECK( y == 4 );
}
