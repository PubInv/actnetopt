// playground.cpp -- Interactive tool for testing configuration of a robotic manipulator via gradient techniquies 
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
#include "playground3d.hpp"
#include <SDL.h>
#include <dlib/optimization.h>
#include <stdio.h>
#include <iostream>
#include <limits.h>
#include "Invert3d.hpp"
// #include "Obstacle.hpp"
#include "Tetrahelix.hpp"
#include <math.h>

// ToDos:
// *) Clean up the code
// *) Implement a circle render for drawing the obstacle
// *) Support multiple obstacles
// *) Create an API-level separation between the playground/rendered and the solver!

using namespace std;
using namespace dlib;


#define USE_DERIVATIVES 1

int debug = 1;

// Sadly, the way the objective and deriviate functions are called
// the do not allow general "client" data to be passed in, and they are static.
// So keeping a global variable is the only effective way to maintain information
// for solving the "forward" problem.
// Therefore "global_truss" is global.
Tetrahelix *Invert::global_truss = 0;

// Obstacle obstacle;

void solve_inverse_problem(Tetrahelix *an) {
  column_vector sp(an->var_edges);
  column_vector lb(an->var_edges);
  column_vector ub(an->var_edges);      

  // We need the Tetrahelix to have distances in order to iniitilize this meaningfully
  for (int i = 0; i < an->var_edges; i++) {
    // This assumes the first edge is fixed, which it is in Tetrahelix
    sp(i) = an->distance(i + 1);
    lb(i) = an->lower_bound(i);
    ub(i) = an->upper_bound(i);    
  }
  
  Invert inv;
  inv.an = an;
  //  inv.set_global_truss(obstacle);
  //  inv.ob = ob;

  int n = an->var_edges;

  if (!USE_DERIVATIVES) {
    // Inv is the objective funciton/object. It computes didstance for goal node.
    find_min_bobyqa(inv, 
		    sp, 
		    (n+1)*(n+2)/2,    // number of interpolation points
		    uniform_matrix<double>(n,1, LOWER_BOUND),  // lower bound constraint
		    uniform_matrix<double>(n,1, UPPER_BOUND),   // upper bound constraint
		    INITIAL/5,    // initial trust region radius (rho_begin)
		    1e-5,  // stopping trust region radius (rho_end)
		    10000   // max number of objective function evaluations
		    );
    for (int i = 0; i < an->var_edges; i++) {
      if (debug)  std::cout << "distance edge" << i+1 << " :  " << sp(i) << std::endl;
      an->distance(i+1) = sp(i);
   }

   if (debug)  std::cout << "inv(x) " << inv(sp) << std::endl;      
  } else {
    best_score = std::numeric_limits<float>::max();

    for (int i = 0; i < an->var_edges; ++i) {
	   best_distances[i] = an->distance(i+1);
    }
    
    double score = find_min_box_constrained(
      			    // bfgs_search_strategy(),
			     lbfgs_search_strategy(30),
			     // cg_search_strategy(),
			     //			     newton_search_strategy,
			     objective_delta_stop_strategy(1e-5),
			     *Invert::objective,
			     *Invert::derivative,
			     sp,
			     uniform_matrix<double>(n,1, LOWER_BOUND),  // lower bound constraint
			     uniform_matrix<double>(n,1, UPPER_BOUND)   // upper bound constraint
			     );

    // I uses this to get rid of the warning
    score = score + 0.0;
    for (int i = 0; i < an->var_edges; ++i) {
      an->distance(i+1) = best_distances[i];
    }
  }

};


// ----------------------------------------------------------------------------------------

int mainx(Tetrahelix *an,column_vector* coords)
{
    try
    {
      // NOTE: This is currently using an algorithm that works without derivatives,
      // even though I think I have correctly analyzed the derivatives (see LaTeX paper.)
      // It can probably be made more efficient to use a dlib algorithm that uses derivatives.
      // However, that is a low priority until I get the playground working.
	{

	  for (int i = 0; i < an->num_edges; ++i) {
	    an->distance(i) = MEDIAN;
	  }

	  // WARNING: We are trying here to skip over fixed edge; this should really be computed!
	  for (int i = 1; i < an->num_edges; ++i) {
	    an->distance(i) = INITIAL;
	  }

	  //	  column_vector* coordsx = new column_vector[an->num_nodes];
	  
	  solve_forward_find_coords(an,coords);	  
	  solve_inverse_problem(an);

	  for (int i = 0; i < an->num_edges; ++i) {
	    //	    std::cout << "mainx" << i << " : " << an->distance(i) << std::endl;
	  }

	  solve_forward_find_coords(an,coords);
	  //	  find_all_coords(an,coords);
	  Invert inv;
	  inv.an = an;
	  //	  inv.set_global_truss(obstacle);
	  column_vector sp(an->var_edges);
	  for (int i = 0; i < an->var_edges; i++) {
	    sp(i) = an->distance(i + 1);
	  }	  
	  // double final = inv(sp);
	  // std::cout << "inv(x) final : " << final << std::endl;
	  // for(int i = 0; i < an->num_nodes; i++) {
	  //   std::cout << " d["<< i << "]" << coords[i](0) << "," << coords[i](1) << std::endl;
	  // }
	  // should now deallocate coords
	}
    }
    catch (std::exception& e)
    {
        cout << e.what() << endl;
    }
    return 0;
}





// TODO: reorganized this so that initialization is in a separate routine

Tetrahelix *init_Tetrahelix() {

  // This needs to be generalized but for now I'm going to
  // just add an obstacle.  If I can successfully work around
  // an obstacle, that will be strong evidence that I have a valuable system.
  // obstacle.radius = 2.0;
  // obstacle.center = column_vector(2);
  // obstacle.center = -2.0, 10.0;
  // obstacle.weight = 4.0;


    Tetrahelix *an = new Tetrahelix(TRUSS_NODES,
		  UPPER_BOUND,
		  LOWER_BOUND,
		  MEDIAN,
		  INITIAL);

  int last_node = an->num_nodes - 1;
  double bx = ((last_node % 2) == 0) ? 0.0 : an->median_d * cos(30.0*M_PI/180);
  double by = (an->median_d/2.0) * last_node;
  // WARNING --- this is almost certainly wrong.
  double bz = 0.0;

  // These are examples of adding different goals...
  //  an->add_goal_node(an->num_nodes*1/3,1*bx/3+-2.0,1*by/3,0.5);
  //  an->add_goal_node(an->num_nodes*2/3,2*bx/3+ 5.0,2*by/3,0.5);  
  double mx = 1.0;
  double my = 1.0;
  double mz = 1.0;
  an->add_goal_node(an->num_nodes-1,bx+mx,by+my,bz+mz,1.0);
  
  if (debug) {
    for(int i = 0; i < an->goal_nodes.size(); i++) {
      cout << "goal_nodes[" << i << "] " << an->goal_nodes[i] << "\n";
    }
  }
  return an;
}



void handle_goal_target_physical(Tetrahelix *an,  column_vector* coordsx,double x, double y, double z) {
  cout << "Interpreted as double!\n";
  column_vector gl(3);
  gl(0) = x;
  gl(1) = y;
  gl(1) = z;
  handle_goal_target(an,coordsx,gl);  
}

void handle_goal_target(Tetrahelix *an,  column_vector* coordsx,column_vector gl) {
  an->goals[an->goals.size() - 1] = gl;
  cout  << "goal : " << gl << "\n";
  best_score = std::numeric_limits<float>::max();	
  auto start = std::chrono::high_resolution_clock::now();
	
  mainx(an,coordsx);
	
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
  long long milliseconds = microseconds/ 1000.0;

  cout << "optimization tim: ms = " << milliseconds << "\n";
  
}

