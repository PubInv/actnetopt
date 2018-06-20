#include "ActNetUtility.hpp"
#include <limits.h>
#include <dlib/geometry.h>

void print_vec(column_vector& vec)
{
  for(int i = 0; i < vec.size(); i++) {
    std::cout << ' ' << vec(i);
    }
    std::cout << '\n';
}


// YUK!  Should not have to define these...
double cot(double x) {
  return 1.0/tan(x);
}

double csc(double x) {
  return 1.0/sin(x);
}
double distance_2d(column_vector a, column_vector b) {
  double d = 0.0;
  for(int i = 0; i < a.size(); i++) {
    d += (a(i)-b(i))*(a(i)-b(i));
  }
  return std::sqrt(d);
}

double distance_3d(column_vector a, column_vector b) {
  double d = 0.0;
  for(int i = 0; i < a.size(); i++) {
    d += (a(i)-b(i))*(a(i)-b(i));
  }
  return std::sqrt(d);
}





double l2_norm(column_vector a) {
  double d = 0.0;
  for(int i = 0; i < a.size(); i++) {
    d += a(i)*a(i);
  }
  return sqrt(d);
}


column_vector cross_product(column_vector a, column_vector b) {
  vector<double,3> va = a;
  vector<double,3> vb = b;
  vector<double,3> vc = va.cross(vb);
  column_vector c(3);
  c(0) = vc(0);
  c(1) = vc(1);
  c(2) = vc(2);
  return c;
}


// https://stackoverflow.com/questions/19350792/calculate-normal-of-a-single-triangle-in-3d-space
// compute the normal (AB rtoated into AC)
column_vector normal(column_vector a, column_vector b, column_vector c) {
  column_vector U = b - a;
  column_vector V = c - a;
  return cross_product(U,V);
}


Chirality tet_chirality(column_vector a, column_vector b, column_vector c, column_vector d)
{
  column_vector N = normal(a,b,c);
  double prod = (N(0)*d(0) + N(1)*d(1) + N(2)*d(2));
  return (prod > 0) ? CCW : CW;
}

column_vector normalized(column_vector a) {
  column_vector b = a / l2_norm(a);
  return b;
}

clock_t time_in_differential = 0;
clock_t time_in_jacobian = 0;
