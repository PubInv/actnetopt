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
  double s_angle_abc0x = -sense2(A(0),A(1),B(0),B(1),C(0),C(1));
  cout << "s_angle_abc0 : " << s_angle_abc0 << "\n";
  BOOST_CHECK( s_angle_abc0 >  0.0 );
  BOOST_CHECK( s_angle_abc0 = s_angle_abc0x);  
  

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

BOOST_AUTO_TEST_CASE( test_dtheta_internal_da )
{
  column_vector A(2);
  column_vector B(2);
  column_vector C(2);
  column_vector D(2);  
  A(0) = 0.0;
  A(1) = 0.0;
  B(0) = 0.0;
  B(1) = 4.0;
  C(0) = 2.0;
  C(1) = 2.0;
  D(0) = 2.0;
  D(1) = 4.0;
  double a = distance_2d(B,C);
  double b = distance_2d(A,C);
  double c = distance_2d(A,B);
  double f = distance_2d(B,D);
  double g = distance_2d(C,D);

    TriLadder tl(LADDER_NODES,
			   UPPER_BOUND,
			   LOWER_BOUND,
			   MEDIAN,
			   INITIAL
	       );

  double dtheta_da = tl.compute_dtheta_internal_da(A,B,C,D,a,b,c,f,g);  
  cout << "dtheta_da degrees: " << dtheta_da*180/M_PI << "\n";
  BOOST_CHECK( dtheta_da > 0.0 );

  // now check symmetry and sign change on other side of y-axis...
  C(0) = -2.0;
  D(0) = -2.0;
  // These actually should not change...
  a = distance_2d(B,C);
  b = distance_2d(A,C);
  c = distance_2d(A,B);
  f = distance_2d(B,D);
  g = distance_2d(C,D);

  
  double dtheta_da_neg = tl.compute_dtheta_internal_da(A,B,C,D,a,b,c,f,g);  
  cout << "dtheta_da_neg degrees: " << dtheta_da_neg*180/M_PI << "\n";
  BOOST_CHECK( dtheta_da_neg < 0.0 );
  BOOST_CHECK( dtheta_da = -dtheta_da_neg );

  B(1) = 3.0;
  C(0) = 2.0;
  D(0) = 2.0;
  
  a = distance_2d(B,C);
  b = distance_2d(A,C);
  c = distance_2d(A,B);
  f = distance_2d(B,D);
  g = distance_2d(C,D);

  double dtheta_da_bal = tl.compute_dtheta_internal_da(A,B,C,D,a,b,c,f,g);
  
  cout << "dtheta_da_bal degrees: " << dtheta_da_bal*180/M_PI << "\n";
  
  BOOST_CHECK( dtheta_da_bal <  dtheta_da );
  BOOST_CHECK( dtheta_da_bal > 0.0 );

  A(0) = 0.0;
  A(1) = 0.0;
  B(0) = 0.0;
  B(1) = 4.0;
  C(0) = 2.0;
  C(1) = 2.0;
  D(0) = 2.0;
  D(1) = 6.0;

  
  a = distance_2d(B,C);
  b = distance_2d(A,C);
  c = distance_2d(A,B);
  f = distance_2d(B,D);
  g = distance_2d(C,D);
  
  double dtheta_da_sym = tl.compute_dtheta_internal_da(A,B,C,D,a,b,c,f,g);
  
  cout << "dtheta_da_sym degrees: " << dtheta_da_sym*180/M_PI << "\n";

  // Now this test a problem I am buggy with. The answer should definitely be positive.

  A(0) = 0.0;
  A(1) = 0.0;
  B(0) = 1.0;
  B(1) = 1.5;
  C(0) = 1.3;
  C(1) = 0.75;
  D(0) = 2.6;
  D(1) = 1.5;

  a = distance_2d(B,C);
  b = distance_2d(A,C);
  c = distance_2d(A,B);
  f = distance_2d(B,D);
  g = distance_2d(C,D);

  double dtheta_da_slnt = tl.compute_dtheta_internal_da(A,B,C,D,a,b,c,f,g);
  
  cout << "dtheta_da_slnt degrees: " << dtheta_da_slnt*180/M_PI << "\n";

  BOOST_CHECK( dtheta_da_slnt > 0.0 );


}

