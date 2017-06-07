'use strict';
var assert = require('assert');

var ANO = require('../actnetopt.js');
var fmin = require('../javascripts/fmin.js');
var algols = require('algorithms').DataStructures;
var THREE = require('../javascripts/three.js');

// console.log(fmin);

var SMALL = 1e-5;

describe('fmin', function() {
    describe('zero function', function() {
	var params = {'maxIterations' : 500, 'history' : []};

	var fxprime = [13,4];
	var f = function(X,fxprime) {
	    var x = X[0], y = X[1];	    	    
	    fxprime = fxprime || [0, 0];	    
	    fxprime[0] = 2*(x-3);
	    fxprime[1] = 2*y
	    return (x - 3)*(x- 3) + y*y + 60;
	}
        var solution = fmin.conjugateGradient(f, fxprime, params);
	console.log(solution);
    });


//     describe('simple trinagle case', function() {
// 	var params = {'maxIterations' : 500, 'history' : []};

// 	/* An attempt to set up the simplest triangle we can 
// for the purpose of testing fmin and our ability to define the gradient 
// satisfactorily. In this model "X" will be a six vector (3 x 2), representing
// 3 x,y variables. G[0] = <0,0>, G[1] = <0,1>, G[2] = < 0.5, sqrt(3)/2>.
// */
// 	var fxprime = [0,0,0,0,0,0];

// 	var H = [0,0,0,1,0.6,Math.sqrt(3)/2];
// 	var G = [[0,1],[1,2],[0,2]];
// 	var f = function(X,fxprime) {
// 	    // We define f to be the sum or the reward function and penalty function
// 	    // Although we could operate on fxprime, I prefer pure functions,
// 	    // so I will define r and p to be pure functions that return the pair [f(X),f'(X)].
// 	    // Note that f(X) is a scalar, and f'(X) is a vector in the same shape as X.
// 	    var rv = r(X,H);
// 	    var pv = p(X,G);
// 	    // against my preferred style, the fmin system requires us to modify the fxprime parameter
// 	    for(var i = 0; i < rv[1].length; i++) {
// 		fxprime[i] = rv[1][i] + pv[1][i];		
// 	    }
// 	    console.log("fxprime",fxprime);
// 	    console.log(rv);
// 	    console.log(pv);
// 	    return rv[0]+pv[0];
// 	}
	
// 	var r = function(X,H) {
// 	    var v = 0;
// 	    H.forEach(function(x,i) {
// 		v += (X[i] - H[i])*(X[i] - H[i]);
// 	    });
// 	    var d = [0,0,0,0,0,0];
// 	    d.forEach(function(e,i) {
// 		d[i] = 2*(X[i] - H[i]);
// 	    });
// 	    console.log("d",d);
// 	    return [v,d];
// 	}
	
// 	var p = function(X,G) {
// 	    var v = 0;
// 	    var d = [0,0,0,0,0,0];
// 	    return [v,d];	    
// 	}

//         var solution = fmin.conjugateGradient(f, fxprime, params);
// 	console.log(solution);
//     });


//     describe('parametric trinagle case', function() {
// 	var params = {'maxIterations' : 50, 'history' : []};

// 	/* An attempt to set up the simplest triangle we can 
// for the purpose of testing fmin and our ability to define the gradient 
// satisfactorily. In this model "X" will be a six vector (3 x 2), representing
// 3 x,y variables. G[0] = <0,0>, G[1] = <0,1>, G[2] = < 0.5, sqrt(3)/2>.
// */
// 	var fxprime = [0,0,0,0,0,0];

// 	var H = [0,0,0,1,0.6,1.1]; // Goal positions.
// 	var E = [[0,1],[1,2],[0,2]]; // Edges
// 	var N = [0,1,2]; // Node
// 	var min = 0.5;
// 	var Y = [min,min,min]; // minimum edge lengths
// 	var max = 1.3;
// 	var Z = [max,max,max]; // maximum edge lengths
// 	var f = function(X,fxprime) {
// 	    // We define f to be the sum or the reward function and penalty function
// 	    // Although we could operate on fxprime, I prefer pure functions,
// 	    // so I will define r and p to be pure functions that return the pair [f(X),f'(X)].
// 	    // Note that f(X) is a scalar, and f'(X) is a vector in the same shape as X.
// 	    var rv = r(X,H);
// 	    var pv = p(X,N,E,Y,Z);
// 	    // against my preferred style, the fmin system requires us to modify the fxprime parameter
// 	    for(var i = 0; i < rv[1].length; i++) {
// 		fxprime[i] = rv[1][i] + pv[1][i];		
// 	    }
// 	    console.log("fxprime",fxprime);
// 	    console.log(rv);
// 	    console.log(pv);
// 	    return rv[0]+pv[0];
// 	}
	
// 	var r = function(X,H) {
// 	    var v = 0;
// 	    H.forEach(function(x,i) {
// 		v += (X[i] - H[i])*(X[i] - H[i]);
// 	    });
// 	    var d = [0,0,0,0,0,0];
// 	    d.forEach(function(e,i) {
// 		d[i] = 2*(X[i] - H[i]);
// 	    });
// 	    console.log("d",d);
// 	    return [v,d];
// 	}
	
// 	var p = function(X,N,E,Y,Z) {
// 	    var v = 0;
// 	    var d = [0,0,0,0,0,0];
	    
// 	    N.forEach(function(n,ix) {
// 		var s = 0;
// 		E.forEach(function(e,n) {
// 		    var i = e[0];
// 		    if (ix == i) { // we only operate on edges related to i
// 			var j = e[1];
// 			var A = new THREE.Vector2(X[i*2],X[i*2+1]);
// 			var B = new THREE.Vector2(X[j*2],X[j*2+1]);
// 			var len = A.distanceTo(B);
// 			var q = 0;
// 			if (len < Y[n]) {
// 			    q += -2;
// 			}
// 			if (len > Z[n]) {
// 			    q += 2;
// 			}
// 			if (q) {
// 			    var temp = ((A.x - B.x) + (A.y - B.y));
// 			    s += q * temp;
// 			    v += temp*temp;
// 			}
// 		    }
// 		});
// 		d[ix] = s;
// 	    });
// //	    console.log("v",v);
// 	    return [v,d];	    
// 	}

//         var solution = fmin.conjugateGradient(f, fxprime, params);
// 	console.log(solution);
//     });


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
	console.log("names",names);
	console.log("X",X);	
	assert(initial.length == X.length);
	assert((free+goal+fixed) == Object.keys(stm.coords).length);
	assert((F.length + X.length) == 2*Object.keys(stm.coords).length);	

	var valueOfNode = function(name,X,F) {
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

	var Y = stm.model.deflb * stm.model.deflb;
	var Z = stm.model.defub * stm.model.defub;
	
	var f = function(X,fxprime) {
	    // We define f to be the sum or the reward function and penalty function
	    // Although we could operate on fxprime, I prefer pure functions,
	    // so I will define r and p to be pure functions that return the pair [f(X),f'(X)].
	    // Note that f(X) is a scalar, and f'(X) is a vector in the same shape as X.
	    console.log("========================");
	    var rv = r(X,stm);
	    var pv = p(F,X,stm,Y,Z);
	    // against my preferred style, the fmin system requires us to modify the fxprime parameter
	    console.log("rv",rv);
	    console.log("pv",pv);	    
	    for(var i = 0; i < rv[1].length; i++) {
		fxprime[i] = rv[1][i] + pv[1][i];
	    }
	    console.log("f,fxprime",rv[0]+pv[0],fxprime);
	    return rv[0]+pv[0];
	}
	
	var r = function(X,stm) {
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
//	    console.log("reward: ",v,d);
	    return [v,d];
	}

	var v = function(nam) {
	};
	
	var p = function(F,X,stm,Y,Z) {
	    const w = 1.0;
	    var v = 0;
	    var nlen = X.length;
	    var d = Array.apply(null, Array(nlen)).map(Number.prototype.valueOf,0);
	    console.log("X",X);
	    // This is wrong because I wrote it as if it was iterating
	    // over nodes, but it is currently iterationg over variables!
	    names.forEach(function (n,ix) {
		var sx = 0;
		var sy = 0;
		const name = n;
		stm.model.g.neighbors(name).forEach(m => {
		    // Note: I am taking some inspiration from mdsGradient in fmin_vis.js
		    var A = valueOfNode(name,X,F);
		    var B = valueOfNode(m,X,F);
		    console.log("name,m,A,B",name,m,A,B);
		    var xi = A.x;
		    var xj = B.x;
		    var yi = A.y;
		    var yj = B.y;
		    var len = A.distanceTo(B);
		    var lensq = len * len;
		    console.log("len,lensq,Y,Z",len,lensq,Y,Z);
		    var q = false;
		    var qsign = 0;n
		    // I think this is wrong...
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
	    v = v ;
//	    console.log("p,d",v,d);
	    return [v,d];	    
	}

	//        var solution = fmin.conjugateGradient(f, initial, params);
        var solution = fmin.conjugateGradient(f, initial, params);	
	console.log("solution",solution);
	console.log(params.history);
	// now for the purpose of checking, we copy our answer back into the model....
	var C = [];
	names.forEach(function (n,ix) {
	    C[n] = new THREE.Vector2(solution.x[ix*2],solution.x[ix*2+1]);
	});
	Object.keys(stm.fixed).forEach(function (n,ix) {
	    C[n] = new THREE.Vector2(stm.coords[n].x,stm.coords[n].y);
	});
	console.log(ANO.legal_configp(stm.model,C));	
    });
    

});

