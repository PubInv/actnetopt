   // Copyright 2017, Robert L. Read

    // This file is part of ActNetOpt.

    // ActNetOpt is free software: you can redistribute it and/or modify
    // it under the terms of the GNU General Public License as published by
    // the Free Software Foundation, either version 3 of the License, or
    // (at your option) any later version.

    // ActNetOpt is distributed in the hope that it will be useful,
    // but WITHOUT ANY WARRANTY; without even the implied warranty of
    // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    // GNU General Public License for more details.

    // You should have received a copy of the GNU General Public License
    // along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

// This file is particularly my attempt to develop an "inverse"
// function which takes as input an actuator model and outputs
// coordinates that match the model.
// This is slightly different than some of the other functions
// in that it starts from lengths. In order to follow the
// language of inverse problems I will call this input an "observation"
// which may be confusing because in general we are specifying this as
// the lengths, not observing it.  Nonetheless this is the "data" rather
// than the "model", the model is a set of nodes matching it.


'use strict';
var assert = require('assert');

var ANO = require('../actnetopt.js');
var opt = require('../javascripts/optimization.js');
var algols = require('algorithms').DataStructures;
var THREE = require('../javascripts/three.js');

// console.log(fmin);

var SMALL = 1e-5;
var SMALL_DIFF = function(a,b) {
    return (Math.abs(a - b) < SMALL);
}

var valueOfNode = function(name,X,F,V) {
    var obj = V[name];
    var type = obj.type;
    var n = obj.index;
    if (type == "fixed") {
	return new THREE.Vector2(F[n*2],F[n*2+1]);
    } else if (type == "goal") {
	return new THREE.Vector2(X[n*2],X[n*2+1]);		
    } else if (type == "free") {
	return new THREE.Vector2(X[n*2],X[n*2+1]);
    } else {
	assert(false);
    }
};

var PENALTY_WEIGHTING_EXP = 2.0;
var LINEAR_PENALTY_WT = 3;
    
var f = function(X,fxprime,stm,Y,Z,names,V,F) {
    // We define f to be the sum or the reward function and penalty function
    // Although we could operate on fxprime, I prefer pure functions,
    // so I will define r and p to be pure functions that return the pair [f(X),f'(X)].
    // Note that f(X) is a scalar, and f'(X) is a vector in the same shape as X.
    //	    console.log("========================");
    var rv = r(X,stm,names,V);
    var pv = p(F,X,stm,Y,Z,V,names,PENALTY_WEIGHTING_EXP,LINEAR_PENALTY_WT);
    // against my preferred style, the fmin system requires us to modify the fxprime parameter
    //	    console.log("rv",rv);
    //	    console.log("pv",pv);	    
    for(var i = 0; i < rv[1].length; i++) {
	fxprime[i] = rv[1][i] + pv[1][i];
    }
    console.log("f,fxprime",rv[0] + pv[0],fxprime);
    return [rv[0]+pv[0],fxprime];
}

var r = function(X,stm,names,V) {
    var v = 0;
    // For each X we need to compute the square of the distance to goals...
    // so we need to know if it is a goal or not.
    // This is incorect; I can't use i for H here.

    var n = X.length;
    var d = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
    
    X.forEach(function(x,i) {
	var name = names[Math.floor(i/2)];
	// is it a goal?
	var obj = V[name];
	if (obj.type == "goal") {
	    // if so we need to process, it contributes to r
	    var gl = null;
	    stm.goals.forEach(g =>
			      {
				  if (g.nd == name) {
				      gl = g.pos;
				  }
			      });

	    assert(gl);
	    var vv = ((i % 2) == 0) ? gl.x : gl.y;
	    v += (X[i] - vv)*(X[i] - vv);
	    d[i] = 2*(X[i] - vv);
	}  else {
	    d[i] = 0; // r does not change as we change this variable
	    v += 0;
	}

    });
    return [v,d];
}

var p = function(F,X,stm,Y,Z,V,names,p_exp,linear_wt) {
    const w = 1.0;
    var v = 0;
    var nlen = X.length;
    var d = Array.apply(null, Array(nlen)).map(Number.prototype.valueOf,0);
    console.log("X",X);
    // This is wrong because I wrote it as if it was iterating
    // over nodes, but it is currently iterationg over variables!
//    console.log("names", names);
    // This is wrong, because we have to include the fixed nodes as well!!!

    var scratch_off = {};
    names.forEach(function (n,ix) {
	var sx = 0;
	var sy = 0;
	const name = n;
	stm.model.g.neighbors(name).forEach(m => {
	    // Note: I am taking some inspiration from mdsGradient in fmin_vis.js
//	    console.log("name,m",name,m);
	    var pair = (name < m) ? name + m : m+name;
	    if (!scratch_off[pair]) {
		scratch_off[pair] = 1;
		var A = valueOfNode(name,X,F,V);
		var B = valueOfNode(m,X,F,V);
		//	    		    console.log("name,m,A,B",name,m,A,B);
		var xi = A.x;
		var xj = B.x;
		var yi = A.y;
		var yj = B.y;
		var len = A.distanceTo(B);
		var lensq = len * len;
		//	    	console.log("len,lensq,Y,Z",len,lensq,Y,Z);
		var q = false;
		var qsign = 0;
		if (lensq < Y) {
		    v += linear_wt * (Y - lensq);
		    q = true;
		    qsign = 1;
		}
		if (lensq > Z) {
		    v += linear_wt * (lensq - Z);
		    q = true;
		    qsign = -1;
		}
		//	    console.log("q,v",q,v);
		if (q) {
		    var tempx = (A.x - B.x);
		    var tempy = (A.y - B.y);
		    sx += -qsign * 2 * linear_wt * tempx;
		    sy += -qsign * 2 * linear_wt * tempy;			    
		}
	    }
	});
//	console.log("sx,sy",sx,sy);		
	d[ix*2] = sx;
	d[ix*2+1] = sy;		
    });
//    console.log("p,d",v,d);
    return [v,d];	    
}

var construct_optimization_model_from_ANO = function(stm) {
    var X = [];
    var initial = [];
    var V = {};
    var fixed = 0,
	free = 0,
	goal = 0;
    var F = [];
    var X = [];
    var names = [];
    var fixed = 0,
	free = 0,
	goal = 0;
    
    Object.keys(stm.coords).forEach(function(n,i) {
	var v = stm.coords[n];
	var isAGoal = false;
	var isFixed = n in stm.fixed;
//	console.log("v[n] = ",v);
	var gpos = null;
	stm.goals.forEach(g =>
			  { if (g.nd == n) {
//			      v = g.pos;
			      gpos = g.pos;
			      isAGoal = true;
			  }
			  });

	assert(!(isAGoal && isFixed));
	var type = isFixed ? "fixed" : (isAGoal ? "goal" : "free");
	var idx = isFixed ? fixed : goal + free;
	V[n] = { "type": type,
		 "index": idx
	       };
//	console.log("type",type);
//	console.log("v = ,gpos",v,gpos);	
	if (type == "fixed") {
	    fixed++;
	    F.push(v.x);
	    F.push(v.y);
	} else if (type == "goal") {
	    // Set up a reverse map so se can get back to a node name
	    // from the index into X.		
	    names[goal+free] = n;		
	    goal++;
	    X.push(gpos.x);
	    X.push(gpos.y);
	    initial.push(v.x);
	    initial.push(v.y);
//	    initial.push(gpos.x);
//	    initial.push(gpos.y);
	} else if (type == "free") {
	    // Set up a reverse map so se can get back to a node name
	    // from the index into X.		
	    names[goal+free] = n;
	    free++;
	    X.push(v.x);
	    X.push(v.y);
	    initial.push(v.x);
	    initial.push(v.y);
	}
    });
    console.log("X,initial",X,initial);
    assert(initial.length == X.length);
    assert((free+goal+fixed) == Object.keys(stm.coords).length);
    assert((F.length + X.length) == 2*Object.keys(stm.coords).length);	

    var Y = stm.model.deflb * stm.model.deflb;
    var Z = stm.model.defub * stm.model.defub;
    return [F,V,X,Y,Z,initial,names,fixed,goal,free];
}

function cartesian_fnc(stm,v) {
    var result = 0.0;
    var names = Object.keys(stm.coords);

    var variables = [];
    names.forEach(nd =>
		  {
		      if (!(nd in stm.fixed)) {
			  variables.push(nd);
		      }
		  });

    // what we really want here is a way to iterate over all edges.
    // We will create a map of the postions from v....
    var p = {};
    var n = 0;
    variables.forEach((nd,ix) =>
		      {
			  var c = new THREE.Vector2(v[ix*2],v[ix*2+1]);
			  p[nd] = c;
		      });
    names.forEach(nd =>
		  {
		      if (!(nd in p)) {
			  var c = ANO.copy_vector(stm.coords[nd]);
			  p[nd] = c;
		      } 
		  });

    stm.cvariables.forEach(
	v0 =>
	    stm.model.g.neighbors(v0).forEach(
		v1 =>
		    {
			var ename = (v0 < v1) ? v0 + ' ' + v1
			    : v1 + ' ' + v0;
			// we need to c0 and c1 corresponding to the positions
			// of these nodes (as definde by the vector v.

			var c0 = p[v0];
			var c1 = p[v1];
			var d = c0.distanceTo(c1);
			result = result + (stm.d[ename] - d)*(stm.d[ename] - d);
		    }
	    )
    );
//    console.log("cartesian_fnc v,result",result,v);
    return result;
}

// Return the coordinates that closely match the model and lengths provided by v
// stm is a model, v is a set of edgelengths corresponding to the d array.
const POWELL = 0;
const L_BFGS = 1;
const CG = 2;
function invert(stm,v,strategy = CG) {
//    console.log(stm,v);

    var X0 = [];
    var names = Object.keys(stm.coords);

    var variables = [];
    names.forEach(nd =>
		  {
		      if (!(nd in stm.fixed)) {
			  variables.push(nd);
		      }
		  });
    
    variables.forEach(nd =>
		      {
			  var p = stm.coords[nd];
			  X0.push(p.x);
			  X0.push(p.y);
		      });
    
    stm.cvariables = variables;

    // now loop over the vector v setting the lengths (d) into the model....
    Object.keys(stm.d).forEach((k,ix) => {
	stm.d[k] = v[ix];
//	console.log("stm.d[k],v[ix]",stm.d[k],v[ix]);
    });
//    console.log("constructed model",stm);

//    console.log("variables", variables);

    //	var solution = opt.minimize_Powell( v => cartesian_fnc(stm,v), X0);
    var grad = function(x) {
	var g = ANO.opt.numerical_gradient(v => cartesian_fnc(stm,v),x);
//	console.log("g == ",x,g);
	return g;
    }

//    var solution = ANO.opt.minimize_GradientDescent(v => cartesian_fnc(stm,v),
//					   grad,
    //					   X0);
    
    var solution;
    if (strategy == POWELL) {
	solution = opt.minimize_Powell(v => cartesian_fnc(stm,v),X0);	
    } else if (strategy == L_BFGS) {
	solution = ANO.opt.minimize_L_BFGS(v => cartesian_fnc(stm,v),
					   grad,
					   X0);
    } else if (strategy == CG) {
      solution = ANO.opt.minimize_GradientDescent(v => cartesian_fnc(stm,v),
					   grad,
					   X0);
    }

//    console.log("stm",stm);
//    console.log("coodinate solution",solution);

    // now for the purpose of checking, we copy our answer back into the model....
    var C = [];
    Object.keys(stm.fixed).forEach(function (n,ix) {
	C[n] = new THREE.Vector2(stm.coords[n].x,stm.coords[n].y);
    });
    variables.forEach(function (n,ix) {
	C[n] = new THREE.Vector2(solution.argument[ix*2],solution.argument[ix*2+1]);
    });
//    console.log("C",C);
    // C is a set of cartesian coordinates!

    //    check_distance_vs_coords(v,names,C,MAX_DIFFERENCE_FROM_INVERSION);
//    console.log("variables :",variables);
//    console.log("C :",C);
//    console.log("v :",v);

    var vnames = [];
    Object.keys(stm.d).forEach(e =>
			     {
				 vnames.push(e);
			     });
//    console.log("vnames :",vnames);
    
    check_distance_vs_coords(v,vnames,C,MAX_DIFFERENCE_FROM_INVERSION);
    var result = check_distance_vs_coords(v,vnames,C,MAX_DIFFERENCE_FROM_INVERSION);
    if (result[0] >= MAX_DIFFERENCE_FROM_INVERSION) {
	console.log("invert failed: ",result);
	console.log("result =",result);
	assert(false);
    }
    
    return C;
}

function compute_goal_penalty_square(m,coords) {
    var penalty = 0;
    m.goals.forEach(
	v0 => {
	    assert(v0.nd in coords);
	    var c0 = coords[v0.nd];
	    var c1 = v0.pos;
	    var d = c0.distanceTo(c1);
	    penalty += d*d;
//	    console.log("XXX c0,c1, penalty",v0,c0,c1,penalty);
	}
    );
    return penalty;
}

function compute_constraint_penalty_square(model,coords) {
    var penalty = 0;
        model.g.vertices.forEach(
	v0 =>
	    model.g.neighbors(v0).forEach(
		v1 =>
		    {
			var c0 = coords[v0];
			var c1 = coords[v1];
			var d = c0.distanceTo(c1);
			var ename = (v0 < v1) ? v0 + ' ' + v1
			    : v1 + ' ' + v0;
			if (d < model.lbs[ename]) {
			    var non_com = Math.abs(model.lbs[ename] - d);
			    penalty += non_com*non_com;			    
			}
			if (d > model.ubs[ename]) {
			    var non_com = Math.abs(model.ubs[ename] - d);
			    penalty += non_com*non_com;
			}
		    }
	    )
	);
    return penalty;
}
const MAX_DIFFERENCE_FROM_INVERSION = 0.2;
function check_distance_vs_coords(v,vnames,coords,limit) {
    // Here we want to iterate over all variables, and assert
    // that the distances between those coords really match...
    var string = "";
    var epsilon = 0.00000000000001;
    var max_non_compliance = 0;
    string += "\n";
    var cnt = 0;
    var max_epsilon = 0;
    vnames.forEach(vn =>
		   {
		       const ns = vn.split(' ');
		       const v0 = ns[0];
		       const v1 = ns[1];
//		       console.log(vn,v0,v1);		       
//		       console.log(coords[v0],coords[v1]);
		       var d = coords[v0].distanceTo(coords[v1]);
		       var epsilon = Math.abs(d - v[cnt]);
		       if (epsilon > max_epsilon) {
			   max_epsilon = epsilon;
		       }
		       if (epsilon > limit) {
			   string += "e: " + epsilon +"\n";
			   string += "v =" + v + "\n";
			   string += "v0 = "+v0 + " " + coords[v0] + "\n";
			   string += "v1 = "+v1 + " " + coords[v1] + "\n";			   
		       }
		       cnt++;
		   }
		  );
    return [epsilon,string];
}

var optimize_goals = function (stm,v,vnames) {
    var result = 0.0;
//    console.log("v = ", v);
    var sgn = v.reduce((a,x) => x < 0 || a,false);
    if (sgn) return 9e4;
    // first, we need to Cartesian coordinates by calling an inversion function...
    // (note: this is a costly act, as it is currently implemented numerically)
    var coords = invert(stm,v);
//    console.log("inverted coords",coords);
    var result = check_distance_vs_coords(v,vnames,coords,MAX_DIFFERENCE_FROM_INVERSION);
    if (result[0] >= MAX_DIFFERENCE_FROM_INVERSION) {
	console.log("invert failed: ",result);
	console.log("result =",result);	
    }
    // Let's prove here that the inversion is close to correct....
    
    // Now we score the coordinates based on reaching the goal...
    var goal_penalty = compute_goal_penalty_square(stm,coords);
    //	    console.log("goal_penalty",goal_penalty,coords);

    // finally, we score based on constraints.....
    var constraint_penalty = compute_constraint_penalty_square(stm.model,coords);
    //	    console.log("constraint_penalty",constraint_penalty);	    
    
    //	    result = goal_penalty + constraint_penalty;
    result = goal_penalty;
    //	    console.log("optimize score",result);
    return result;
};

describe('invert', function() {
    it('We can invert from a long distance',function () {
	var m = ANO.simple_triangle_problem();

	var X0 = [];
	var variables = [];
	var vnames = [];

	// NOTE: CRIITICAL! The fundamental problem is that if this goal position below
	// is very far away from the optimal solution, I get no convergence at all,
	// even though I believe the space of optimization to be smooth and simple.
	m.goals[0] = { nd: 'c',
		       // Note: 3,3 fails!
		       pos: new THREE.Vector2(3,3),
		       wt: 3 };

	m.d = ANO.create_lengths_object(m.model,m.nodeset,1.5);
	console.log(m);
	console.log("ZZZZZZZ");

	Object.keys(m.d).forEach(e =>
		      {
			  vnames.push(e);
			  variables.push(m.d[e]);
		      });

//	console.log("variables",variables);

	var cvariables = variables.slice();
//	console.log("cvariables",cvariables);	
	const start_ms = new Date().getTime();
	var cnt = 0;
	const X_LIM = 36;
	const X_STEP = 3;
	const Y_LIM = 12;
	const Y_STEP = 3;
	for(var i = 0; i < X_LIM; i += X_STEP) {
	    for(var j = 0; j < Y_LIM; j += Y_STEP) {
	    m.goals[0] = { nd: 'c',
			   pos: new THREE.Vector2(i,j),
			   wt: 3 };

	    // This is all wrong, we need to set up distances based on coordinates
	    // and make sure we can recover!


	    // This is the "b c" distance
	    variables[0] = m.coords['a'].distanceTo(m.coords['b']);	    	    	    
	    variables[1] = m.goals[0].pos.distanceTo(m.coords['a']);
	    variables[2] = m.goals[0].pos.distanceTo(m.coords['b']);


//	    console.log("cvariables XXXX",cvariables);	    
//	    variables = cvariables.slice();
//	    console.log("variables XXXX",variables);
	    m.d = ANO.create_lengths_object(m.model,m.nodeset,1.5);	    
		//		var coords = invert(m,variables,L_BFGS);
		

		// Note: At present L_BFGS doesn't work.
		// CG and POWELL seem to work
		// const strategy = POWELL; // Note: At present L_BFGS doesn't work.
		const strategy = CG; 
		var coords = invert(m,variables,strategy);		
	    var result = check_distance_vs_coords(variables,
						  vnames,
						  coords,
						  MAX_DIFFERENCE_FROM_INVERSION);
//	    console.log(i,result[0],coords);
	    var rcs = [];
	    rcs[0] = coords['a'].distanceTo(coords['b']);
	    rcs[1] = coords['a'].distanceTo(coords['c']);
	    rcs[2] = coords['b'].distanceTo(coords['c']);	    	    
//	    console.log(rcs);
	    var diffs = [];
	    diffs[0] = rcs[0]-variables[0];
	    diffs[1] = rcs[1]-variables[1];
	    diffs[2] = rcs[2]-variables[2];
		console.log("diffs",i,j,diffs);
	    cnt++;
	    if (result[0] >= MAX_DIFFERENCE_FROM_INVERSION) {
		console.log("invert failed: ",result);
		
		assert(false);
	    }
	    }
	}
	const finish_ms = new Date().getTime();
	console.log("mean inversion time (ms) :",(finish_ms - start_ms) / cnt);
    });
});
      
describe('optimize', function() {
    it('optimize_goals is reasonable',function () {
	var m = ANO.simple_triangle_problem();

	var X0 = [];
	var variables = [];
	var vnames = [];

	// NOTE: CRIITICAL! The fundamental problem is that if this goal position below
	// is very far away from the optimal solution, I get no convergence at all,
	// even though I believe the space of optimization to be smooth and simple.
	m.goals[0] = { nd: 'c',
		       // Note: 3,3 fails!
		       pos: new THREE.Vector2(3,3),
		       wt: 3 };


	m.d = ANO.create_lengths_object(m.model,m.nodeset,1.5);

	Object.keys(m.d).forEach(e =>
		      {
			  vnames.push(e);
			  variables.push(m.d[e]);
		      });

	var coords = invert(m,variables);

	var cur = 0; 
	for(var i = 3; i < 20; i++) {
	    coords['c'] = new THREE.Vector2(i,3);
	    var goal = compute_goal_penalty_square(m,coords);
	    assert(goal >= cur,"goal "+goal+" cur "+cur);
	    coords['c'] = new THREE.Vector2(3,i);
	    var goal = compute_goal_penalty_square(m,coords);
	    assert(goal >= cur,"goal "+goal+" cur "+cur);	    	    
//	    console.log("goal:",coords['c'],i,goal);
	}

	
	console.log("variables", variables);
 	m.cvariables = variables;		
    }); 

    it('goal_penalty goes down as we lengthen towards far goal',function() {
	var m = ANO.simple_triangle_problem();

	var X0 = [];
	var variables = [];
	var vnames = [];

	m.goals[0] = { nd: 'c',
		       pos: new THREE.Vector2(10.0,10.0),
		       wt: 3 };



	function set_to_x(x) {
	    m.d = ANO.create_lengths_object(m.model,m.nodeset,x);
	}
	for(var x = 3.0; x < 5.0; x++) {
	    set_to_x(x);
	    variables = [];
	    vnames = [];
	    Object.keys(m.d).forEach(e =>
				     {
					 vnames.push(e);
					 variables.push(m.d[e]);
				     });
	    var coords = invert(m,variables);
	    var goal = compute_goal_penalty_square(m,coords);
	    console.log("GOAL =",x,coords,variables,goal);
	}

    });

    it('we can optimize based on inversion', function() {

	var m = ANO.simple_triangle_problem();

	var X0 = [];
	var variables = [];
	var vnames = [];

	// NOTE: CRIITICAL! The fundamental problem is that if this goal position below
	// is very far away from the optimal solution, I get no convergence at all,
	// even though I believe the space of optimization to be smooth and simple.

	m.goals[0] = { nd: 'c',
		       // Note: 3,3 fails!
		       // Note going even as high as 6 makes inversion fail.
		       pos: new THREE.Vector2(3,3),
		       wt: 3 };

	m.d = ANO.create_lengths_object(m.model,m.nodeset,1.5);

	Object.keys(m.d).forEach(e =>
		      {
			  vnames.push(e);
			  variables.push(m.d[e]);
			  });
//	m.lvariables = variables;
	
	console.log("variables", variables);
	console.log("vanems",vnames);
 	m.cvariables = variables;		

	var grad_opt = function(x) {
	    var g = ANO.opt.numerical_gradient(v => optimize_goals(m,v,vnames),x);
	    return g;
	}

//	var solution = ANO.opt.minimize_L_BFGS(v => optimize_goals(m,v,vnames),
//				       grad_opt,
//    					       variables);

//    var solution = ANO.opt.minimize_GradientDescent(v => optimize_goals(m,v,vnames),
//						    grad_opt,
//						    variables);
    
	var solution = opt.minimize_Powell(v => optimize_goals(m,v,vnames),variables);
    
	console.log("FINAL solution",solution);
//	console.log("FINAL m",m.d);
	variables = [];
	vnames = [];
	Object.keys(m.d).forEach(e =>
		      {
			  vnames.push(e);
			  variables.push(m.d[e]);
			  });
	var final_c = invert(m,variables)
	console.log("final coords",final_c);
	console.log(ANO.max_non_compliant_debug(m.model,final_c));
    });

});

