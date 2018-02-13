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
  int d;
  if ((e % 3) == 0) {
    d = 1;
  }
  if ((e % 3) == 1) {
    d = 2;
  }
  if ((e % 3) == 2) {
    d = 3;
  }
  int s = small_node(e);
  return s + d;
}

int Tetrahelix::small_node(int e) {
  return e / 3;
}



// what did I mean by this name?
// Number of edges before?
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
	return 2;
    }
  } else if (x == 1) {
      if (y == 2)
	return 3;
      else {
	if (y == 3)
	  return 4;
	else if (y == 4)
	  return 5;
      }
  } else {
      int d = y - x;
      cout << " final d " << d << "\n";
      return x*3+d-1;
  }
  abort();
}


  int Tetrahelix::edge_between(int x,int y) {
  if (abs(x-y) > 3) {
    cout << "x y : " << x << " " << y << "\n";
    abort();
    return -1;
  } else if (x == y) {
    cout << "x y : " << x << " " << y << "\n";    
    abort();    
    return -1;
  } else return (x < y) ? edge_between_aux(x,y) : edge_between_aux(y,x);
}

Tetrahelix::Tetrahelix(int nodes,
		     double u,
		     double l,
		     double m,
		     double i)
{
  upper_bound_d = u;
  lower_bound_d = l;
  median_d = m;
  initial_d = i;

    num_nodes = nodes;
    
    num_edges = edges_in_tetrahelix(num_nodes);
    
    var_edges = num_edges-1;
    node_fixing_order = new int[num_nodes];
    coords = new column_vector[num_nodes];      

    fixed_nodes.set_size(2);
    fixed_nodes(0) = A;
    fixed_nodes(1) = B;

    // add the edges to the graph object
    distance.set_size(num_edges);
    lower_bound.set_size(num_edges);
    upper_bound.set_size(num_edges);    
    for (int i = 0; i < num_edges; ++i) {
      distance(i) = median_d;
    }
    
    for (int i = 0; i < var_edges; ++i) {
      lower_bound(i) = lower_bound_d;
      upper_bound(i) = upper_bound_d;            
    }
  }
