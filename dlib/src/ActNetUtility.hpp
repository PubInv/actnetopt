// ActNetUtility.hpp -- Universal utility functions for ActNetOptimization
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

#ifndef ACTNETUTILITY_HPP
#define ACTNETUTILITY_HPP 1


#include <dlib/geometry.h>


using namespace dlib;

typedef matrix<double,0,1> column_vector;

typedef enum { CW, CCW } Chirality;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


template <typename T>
bool is_close_to_zero(T x)
{
    return std::abs(x) < std::numeric_limits<T>::epsilon();
}

void print_vec(column_vector& vec);


// YUK!  Should not have to define these...
double cot(double x);

double csc(double x);
double distance_2d(column_vector a, column_vector b);
double distance_3d(column_vector a, column_vector b);
double l2_norm(column_vector a);

column_vector cross_product(column_vector a, column_vector b);

column_vector normal(column_vector a, column_vector b, column_vector c);

Chirality tet_chirality(column_vector a, column_vector b, column_vector c, column_vector d);

#endif
