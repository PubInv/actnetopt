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

#include "Tetrahelix.hpp"

using namespace std;
using namespace dlib;



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
  return small_node(e) + (((e % 2) == 0) ? 1 : 2);
}

int Tetrahelix::small_node(int e) {
  return e / 2;
}



// what did I mean by this name?
int neb(int x) {
  if (x == 0) return 0;
  if (x == 1) return 1;
  if (x == 2) return 3;
  if (x >= 3) return 3*(x-1);
  abort();
}

int edge_between_aux(int x,int y) {
  if (x == 0) {
    if (y == 1) return 0;
    else {
      if (y == 2) return 1;
      else if (y == 3)
	return 3;
    }
  } else if (x == 1) {
      if (y == 2)
	return 2;
      else {
	if (y == 3)
	  return 4;
	else if (y == 4)
	  return 6;
      }
  } else {
      int d = y - x;
      int n = neb(y - 1);
      if (d == 1) return 2+n;
      else {
	if (d == 2) return 1+n;
	else if (d == 3)
	  return 0+n;
      }
  }
  abort();
}


  int Tetrahelix::edge_between(int x,int y) {
  if (abs(x-y) > 3) {
    return -1;
  } else if (x == y) {
    return -1;
  } else return (x < y) ? edge_between_aux(x,y) : edge_between_aux(y,x);
}
