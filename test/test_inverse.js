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
	console.log("v[n] = ",v);
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
	console.log("type",type);
	console.log("v = ,gpos",v,gpos);	
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


describe('inverse', function() {
    it('simply can find inverse', function() {

    	var stm = ANO.create_standard_observation(10);

	console.log(stm);

	// Since we are trying to use Powell's Method,
	// we just use a straight penalty function, but
	// we do rather need to know about the model...


	// Optmize.js wants a linear array of variables,
	// in this case that is positions for the non-fixed points in order.
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
	
	stm.variables = variables;
	
	var test_fnc = function (stm,v) {
	    var result = 0.0;
//	    console.log("v = ", v);

	    // now we need to iterate over the vector, but also
	    // find all of the connections, which will require to check
	    // the graph.
//	    var testv = 3; // this will try to drive the values to 3...just a test!
//	    for (var i = 0; i < v.length; i++){
//		result = result + (v[i] - testv) * (v[i] - testv);
//	    }

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
//				  console.log("nd,stm.coords[nd]",nd,stm.coords[nd]);
				  var c = ANO.copy_vector(stm.coords[nd]);
				  p[nd] = c;
			      } 
			  });
//	    console.log("p =",p);
	    

	    stm.variables.forEach(
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
//				console.log("ename,desired,actual",ename,stm.d[ename],d);
//				console.log("root of distance",ename,(stm.d[ename] - d)*(stm.d[ename] - d),c0,c1);
				result = result + (stm.d[ename] - d)*(stm.d[ename] - d);
			    }
		    )
	    );
//	    console.log("result",result);
	    return result;
	};
	console.log("variables", variables);

//	var solution = opt.minimize_Powell( v => test_fnc(stm,v), X0);
	var grad = function(x) {
	    var g = ANO.opt.numerical_gradient(v => test_fnc(stm,v),x);
//	    console.log("g == ",x,g);
	    return g;
	}

	var solution = ANO.opt.minimize_L_BFGS(v => test_fnc(stm,v),
					       grad,
					       X0);	

	console.log("stm",stm);
	console.log("solution",solution);

	// now for the purpose of checking, we copy our answer back into the model....
	var C = [];
	Object.keys(stm.fixed).forEach(function (n,ix) {
	    C[n] = new THREE.Vector2(stm.coords[n].x,stm.coords[n].y);
	});
	variables.forEach(function (n,ix) {
	    C[n] = new THREE.Vector2(solution.argument[ix*2],solution.argument[ix*2+1]);
	});
	console.log("C",C);

	console.log(ANO.max_non_compliant_debug(stm.model,C));
	var mc = ANO.max_non_compliant(stm.model,C);
	assert(mc < 0.05,"non_compliance: "+mc);
    });
    

});

