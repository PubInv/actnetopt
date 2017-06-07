'use strict';
var assert = require('assert');

var ANO = require('../actnetopt.js');
var opt = require('../javascripts/optimization.js');
var algols = require('algorithms').DataStructures;
var THREE = require('../javascripts/three.js');

// console.log(fmin);

var SMALL = 1e-5;

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


var f = function(X,fxprime,stm,Y,Z,names,V,F) {
    // We define f to be the sum or the reward function and penalty function
    // Although we could operate on fxprime, I prefer pure functions,
    // so I will define r and p to be pure functions that return the pair [f(X),f'(X)].
    // Note that f(X) is a scalar, and f'(X) is a vector in the same shape as X.
    //	    console.log("========================");
    var rv = r(X,stm,names,V);
    var pv = p(F,X,stm,Y,Z,V,names);
    // against my preferred style, the fmin system requires us to modify the fxprime parameter
    //	    console.log("rv",rv);
    //	    console.log("pv",pv);	    
    for(var i = 0; i < rv[1].length; i++) {
	fxprime[i] = rv[1][i] + pv[1][i];
    }
    //	    console.log("f,fxprime",rv[0]+pv[0],fxprime);
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

var p = function(F,X,stm,Y,Z,V,names) {
    const w = 1.0;
    var v = 0;
    var nlen = X.length;
    var d = Array.apply(null, Array(nlen)).map(Number.prototype.valueOf,0);
//    console.log("X",X);
    // This is wrong because I wrote it as if it was iterating
    // over nodes, but it is currently iterationg over variables!
    names.forEach(function (n,ix) {
	var sx = 0;
	var sy = 0;
	const name = n;
	stm.model.g.neighbors(name).forEach(m => {
	    // Note: I am taking some inspiration from mdsGradient in fmin_vis.js
	    var A = valueOfNode(name,X,F,V);
	    var B = valueOfNode(m,X,F,V);
	    //		    console.log("name,m,A,B",name,m,A,B);
	    var xi = A.x;
	    var xj = B.x;
	    var yi = A.y;
	    var yj = B.y;
	    var len = A.distanceTo(B);
	    var lensq = len * len;
	    //		    console.log("len,lensq,Y,Z",len,lensq,Y,Z);
	    var q = false;
	    var qsign = 0;n
	    if (lensq < Y) {
		v += (Y - lensq);
		q = true;
		qsign = 1;
	    }
	    if (lensq > Z) {
		v += (lensq - Z);
		q = true;
		qsign = -1;
	    }
	    //		    console.log("q,v",q,v);
	    if (q) {
		var tempx = (A.x - B.x);
		var tempy = (A.y - B.y);
		sx += qsign * 2 * tempx;
		sy += qsign * 2 * tempy;			    
	    }
	});
	//		console.log("sx,sy",sx,sy);		
	d[ix*2] = sx;
	d[ix*2+1] = sy;		
    });
    //	    console.log("p,d",v,d);
    return [v,d];	    
}


describe('fmin', function() {
    // describe('zero function', function() {
    // 	var params = {'maxIterations' : 500, 'history' : []};

    // 	var initial = [13,4];
    // 	var fq = function(X,fxprime) {
    // 	    var x = X[0], y = X[1];	    	    
    // 	    fxprime = fxprime || [0, 0];	    
    // 	    fxprime[0] = 2*(x-3);
    // 	    fxprime[1] = 2*y
    // 	    return (x - 3)*(x- 3) + y*y + 60;
    // 	}
    // 	var grd = function(X) {
    // 	    var n = X.length;
    // 	    var fxprime = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
    // 	    fq(X,fxprime);
    // 	    return fxprime;
    // 	}
    // 	//      var solution = opt.minimize_Powell(f, initial);
    // 	var solution = opt.minimize_L_BFGS(f,grd, initial);	
    // 	console.log(solution);
    // });

    describe('I am computing the derivative correctly', function() {

    	var stm = ANO.simple_triangle_problem();
	
	// Now I will attempt to integrate the mathematical approaches here...
	
	//	var H = [0,0,0,1,0.6,1.1]; // Goal positions.

	// Nodes are partitioned into three classes:
	// fixed, free, and goals.
	var X = [];
	var X0 = [];
	var N = []; // Node

	var initial = [];
	var V = {};
	var fixed = 0,
	    free = 0,
	    goal = 0;
	var F = [];
	var X = [];
	var names = [];
	Object.keys(stm.coords).forEach(function(n,i) {
	    var v = stm.coords[n];
	    var isAGoal = false;
	    var isFixed = n in stm.fixed;
	    stm.goals.forEach(g =>
			      { if (g.nd == n) {
				  v = g.pos;
				  isAGoal = true;
			      }
			      });
	    assert(!(isAGoal && isFixed));
	    var type = isFixed ? "fixed" : (isAGoal ? "goal" : "free");
	    var idx = isFixed ? fixed : goal + free;
	    V[n] = { "type": type,
		     "index": idx
		   };
	    if (type == "fixed") {
		fixed++;
		F.push(v.x);
		F.push(v.y);
	    } else if (type == "goal") {
	    // Set up a reverse map so se can get back to a node name
	    // from the index into X.		
		names[goal+free] = n;		
		goal++;
		X.push(v.x/3);
		X.push(v.y/3);
		initial.push(v.x/3);
		initial.push(v.y/3);
	    } else if (type == "free") {
	    // Set up a reverse map so se can get back to a node name
	    // from the index into X.		
		names[goal+free] = n;
		free++;
		X.push(v.x/3);
		X.push(v.y/3);
		initial.push(v.x/3);
		initial.push(v.y/3);
	    }
	});
	assert(initial.length == X.length);
	assert((free+goal+fixed) == Object.keys(stm.coords).length);
	assert((F.length + X.length) == 2*Object.keys(stm.coords).length);	

	var Y = stm.model.deflb * stm.model.deflb;
	var Z = stm.model.defub * stm.model.defub;

	var rd = function(X) {
	    return r(X,stm,names,V)[1];
	}
	var rv = function(X) {
	    return r(X,stm,names,V)[0];
	}

	var grd_r0 = function(X) {
//	    var n = X.length;
//	    var fxprime = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
	    var v = rd(X);
	    return v;
	}

	console.log("computing r0");
	var initial_copy = initial.slice();
	const r0v = grd_r0(initial);
	const r1v = opt.numerical_gradient(rv,initial_copy);
	
	console.log("r0v,r1v",r0v,r1v);
	
	console.log("i","r comp","r numer","==");
	for(var i = 0; i < r0v.length; i++) {
	    console.log(i,r0v[i],r1v[i],r0v[i] == r1v[i]);
	};

	
	var pd = function(X) {
	    return p(F,X,stm,Y,Z,V,names)[1];
	}
	var pv = function(X) {
	    return p(F,X,stm,Y,Z,V,names)[0];
	}

	var grd_p0 = function(X) {
//	    var n = X.length;
//	    var fxprime = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
	    var v = pd(X);
	    return v;
	}
	
	console.log("computing p0");
	var initial_copy = initial.slice();
	const p0v = grd_p0(initial);
	const p1v = opt.numerical_gradient(pv,initial_copy);
	
	console.log("p0v,p1v",p0v,p1v);
	
	console.log("i","r comp","r numer","==");
	
	for(var i = 0; i < p0v.length; i++) {
	    console.log(i,p0v[i],p1v[i],p0v[i] == p1v[i]);
	};


	console.log("XXXXXXXXXXXXXXXXXXXXXXXXXX");

/*	var fd = function(X,fxprime) {
	    return f(X,fxprime,stm,Y,Z,names,V,F)[1];	    
	}
*/
	
	var fv = function(X) {
	    var n = X.length;
	    var fxprime = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
	    return f(X,fxprime,stm,Y,Z,names,V,F)[0];	    
	}

	var fd = function(X) {
	    var n = X.length;
	    var fxprime = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
	    return f(X,fxprime,stm,Y,Z,names,V,F)[1];	    
	}
	
	var grd0 = function(X) {
	    var n = X.length;
	    var fxprime = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
	    console.log("FV FXPRIME",fxprime);	    
	    var v = fd(X,fxprime);
	    return v;
	}
	
	console.log("Computing GRD");
	const grd0v = grd0(initial);
	const grd1v =  opt.numerical_gradient(fv,initial);
	console.log("grd0v",grd0v);
	console.log("grd1v",grd1v);
	console.log("i","comp","numer","==");
	for(var i = 0; i < grd0v.length; i++) {
	    console.log(i,grd0v[i],grd1v[i], grd0v[i] == grd1v[i]);
	};
	
    }
	    );

    describe('fmin used directoly for simple problem', function() {
	var params = {'maxIterations' : 5, 'history' : []};

	/* An attempt to set up the simplest triangle we can 
for the purpose of testing fmin and our ability to define the gradient 
satisfactorily. In this model "X" will be a six vector (3 x 2), representing
3 x,y variables. G[0] = <0,0>, G[1] = <0,1>, G[2] = < 0.5, sqrt(3)/2>.
	*/

    	var stm = ANO.simple_triangle_problem();

	// Now I will attempt to integrate the mathematical approaches here...
	
	//	var H = [0,0,0,1,0.6,1.1]; // Goal positions.

	// Nodes are partitioned into three classes:
	// fixed, free, and goals.
	var X = [];
	var X0 = [];
	var N = []; // Node

	var initial = [];
	var V = {};
	var fixed = 0,
	    free = 0,
	    goal = 0;
	var F = [];
	var X = [];
	var names = [];
	Object.keys(stm.coords).forEach(function(n,i) {
	    var v = stm.coords[n];
	    var isAGoal = false;
	    var isFixed = n in stm.fixed;
	    stm.goals.forEach(g =>
			      { if (g.nd == n) {
				  v = g.pos;
				  isAGoal = true;
			      }
			      });
	    assert(!(isAGoal && isFixed));
	    var type = isFixed ? "fixed" : (isAGoal ? "goal" : "free");
	    var idx = isFixed ? fixed : goal + free;
	    V[n] = { "type": type,
		     "index": idx
		   };
	    if (type == "fixed") {
		fixed++;
		F.push(v.x);
		F.push(v.y);
	    } else if (type == "goal") {
	    // Set up a reverse map so se can get back to a node name
	    // from the index into X.		
		names[goal+free] = n;		
		goal++;
		X.push(v.x/3);
		X.push(v.y/3);
		initial.push(v.x/3);
		initial.push(v.y/3);
	    } else if (type == "free") {
	    // Set up a reverse map so se can get back to a node name
	    // from the index into X.		
		names[goal+free] = n;
		free++;
		X.push(v.x/3);
		X.push(v.y/3);
		initial.push(v.x/3);
		initial.push(v.y/3);
	    }
	});
	assert(initial.length == X.length);
	assert((free+goal+fixed) == Object.keys(stm.coords).length);
	assert((F.length + X.length) == 2*Object.keys(stm.coords).length);	


	var Y = stm.model.deflb * stm.model.deflb;
	var Z = stm.model.defub * stm.model.defub;


	var fv = function(X) {
	    var n = X.length;
	    var fxprime = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
	    return f(X,fxprime,stm,Y,Z,names,V,F)[0];	    
	}
	var fd = function(X) {
	    var n = X.length;
	    var fxprime = Array.apply(null, Array(n)).map(Number.prototype.valueOf,0);
	    return f(X,fxprime,stm,Y,Z,names,V,F)[1];	    
	}
	
	var grd = function(X) {
	    console.log("grd0",grd0(X));
	    const grd1 =  opt.numerical_gradient(f0,X);
	    console.log("grd1",grd1);
	    return grd1;
	}

	

	//        var solution = fmin.conjugateGradient(f, initial, params);
//        var solution = fmin.conjugateGradient(f, initial, params);
	//	var solution = opt.minimize_Powell(f,initial);
	var solution = opt.minimize_L_BFGS(fv,fd,initial);	
	console.log("solution",solution);
///	console.log(params.history);
	// now for the purpose of checking, we copy our answer back into the model....
	var C = [];
	names.forEach(function (n,ix) {
	    C[n] = new THREE.Vector2(solution.argument[ix*2],solution.argument[ix*2+1]);
	});
	Object.keys(stm.fixed).forEach(function (n,ix) {
	    C[n] = new THREE.Vector2(stm.coords[n].x,stm.coords[n].y);
	});
	console.log(ANO.legal_configp(stm.model,C));	
    });
    

});

