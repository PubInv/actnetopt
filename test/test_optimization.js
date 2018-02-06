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


describe('optimization', function() {
    it('optimizes a medium problem', function() {
	var params = {'maxIterations' : 5, 'history' : []};

    	var stm = ANO.medium_triangle_problem();
	var om = construct_optimization_model_from_ANO(stm);
	
	var F = om[0];
	var V = om[1];
//	var X = om[2];
	var Y = om[3];
	var Z = om[4];
	var initial = om[5];
	var names = om[6];
	var fixed = om[7];
	var goal = om[8];
	var free = om[9];

	console.log(stm);
	console.log("om",om);

	var fvn = function(X,n) {
	    var fxprime = Array.apply(null, Array(X.length)).map(Number.prototype.valueOf,0);
	    return f(X,fxprime,stm,Y,Z,names,V,F)[n];	    
	}

	var fv = function(X) {
	    return fvn(X,0);
	}
	var fd = function(X) {
	    return fvn(X,1);
	}

	//        var solution = fmin.conjugateGradient(f, initial, params);
//        var solution = fmin.conjugateGradient(f, initial, params);
//	var solution = opt.minimize_Powell(fv,initial);
	var solution = opt.minimize_L_BFGS(fv,fd,initial);

	console.log("stm",stm);
	console.log("solution",solution);
///	console.log(params.history);
	// now for the purpose of checking, we copy our answer back into the model....
	var C = [];
	Object.keys(stm.fixed).forEach(function (n,ix) {
	    C[n] = new THREE.Vector2(stm.coords[n].x,stm.coords[n].y);
	});
	names.forEach(function (n,ix) {
	    C[n] = new THREE.Vector2(solution.argument[ix*2],solution.argument[ix*2+1]);
	});
	console.log("C",C);
//	console.log(ANO.legal_configp(stm.model,C));
	console.log(ANO.max_non_compliant(stm.model,C));
	var mc = ANO.max_non_compliant(stm.model,C);
	assert(mc < 0.05,"non_compliance: "+mc);
    });
    

});

