'use strict';
var assert = require('assert');

var ANO = require('../actnetopt.js');
var fmin = require('../javascripts/fmin.js');
var algols = require('algorithms').DataStructures;
var THREE = require('../javascripts/three.js');

// console.log(fmin);

var SMALL = 1e-5;

describe('fmin', function() {
/*    describe('zero function', function() {
	var params = {'maxIterations' : 500, 'history' : []};

	var fxprime = [12,12];
	var f = function(X,fxprime) {
	    var x = X[0], y = X[1];	    	    
	    fxprime = fxprime || [0, 0];	    
	    fxprime[0] = 2*(x-4);
	    fxprime[1] = 2*y
	    return (x - 4)*(x- 4) + y*y + 60;
	}
        var solution = fmin.conjugateGradient(f, fxprime, params);
	console.log(solution);
    });

*/
    describe('simple trinagle case', function() {
	var params = {'maxIterations' : 500, 'history' : []};

	/* An attempt to set up the simplest triangle we can 
for the purpose of testing fmin and our ability to define the gradient 
satisfactorily. In this model "X" will be a six vector (3 x 2), representing
3 x,y variables. G[0] = <0,0>, G[1] = <0,1>, G[2] = < 0.5, sqrt(3)/2>.
*/
	var fxprime = [0,0,0,0,0,0];

	var H = [0,0,0,1,0.6,Math.sqrt(3)/2];
	var G = [[0,1],[1,2],[0,2]];
	var f = function(X,fxprime) {
	    // We define f to be the sum or the reward function and penalty function
	    // Although we could operate on fxprime, I prefer pure functions,
	    // so I will define r and p to be pure functions that return the pair [f(X),f'(X)].
	    // Note that f(X) is a scalar, and f'(X) is a vector in the same shape as X.
	    var rv = r(X,H);
	    var pv = p(X,G);
	    // against my preferred style, the fmin system requires us to modify the fxprime parameter
	    for(var i = 0; i < rv[1].length; i++) {
		fxprime[i] = rv[1][i] + pv[1][i];		
	    }
	    console.log("fxprime",fxprime);
	    console.log(rv);
	    console.log(pv);
	    return rv[0]+pv[0];
	}
	
	var r = function(X,H) {
	    var v = 0;
	    H.forEach(function(x,i) {
		v += (X[i] - H[i])*(X[i] - H[i]);
	    });
	    var d = [0,0,0,0,0,0];
	    d.forEach(function(e,i) {
		d[i] = 2*(X[i] - H[i]);
	    });
	    console.log("d",d);
	    return [v,d];
	}
	
	var p = function(X,G) {
	    var v = 0;
	    var d = [0,0,0,0,0,0];
	    return [v,d];	    
	}

        var solution = fmin.conjugateGradient(f, fxprime, params);
	console.log(solution);
    });


    describe('parametric trinagle case', function() {
	var params = {'maxIterations' : 50, 'history' : []};

	/* An attempt to set up the simplest triangle we can 
for the purpose of testing fmin and our ability to define the gradient 
satisfactorily. In this model "X" will be a six vector (3 x 2), representing
3 x,y variables. G[0] = <0,0>, G[1] = <0,1>, G[2] = < 0.5, sqrt(3)/2>.
*/
	var fxprime = [0,0,0,0,0,0];

	var H = [0,0,0,1,0.6,1.1]; // Goal positions.
	var E = [[0,1],[1,2],[0,2]]; // Edges
	var N = [0,1,2]; // Node
	var min = 0.5;
	var Y = [min,min,min]; // minimum edge lengths
	var max = 1.3;
	var Z = [max,max,max]; // maximum edge lengths
	var f = function(X,fxprime) {
	    // We define f to be the sum or the reward function and penalty function
	    // Although we could operate on fxprime, I prefer pure functions,
	    // so I will define r and p to be pure functions that return the pair [f(X),f'(X)].
	    // Note that f(X) is a scalar, and f'(X) is a vector in the same shape as X.
	    var rv = r(X,H);
	    var pv = p(X,N,E,Y,Z);
	    // against my preferred style, the fmin system requires us to modify the fxprime parameter
	    for(var i = 0; i < rv[1].length; i++) {
		fxprime[i] = rv[1][i] + pv[1][i];		
	    }
	    console.log("fxprime",fxprime);
	    console.log(rv);
	    console.log(pv);
	    return rv[0]+pv[0];
	}
	
	var r = function(X,H) {
	    var v = 0;
	    H.forEach(function(x,i) {
		v += (X[i] - H[i])*(X[i] - H[i]);
	    });
	    var d = [0,0,0,0,0,0];
	    d.forEach(function(e,i) {
		d[i] = 2*(X[i] - H[i]);
	    });
	    console.log("d",d);
	    return [v,d];
	}
	
	var p = function(X,N,E,Y,Z) {
	    var v = 0;
	    var d = [0,0,0,0,0,0];
	    
	    N.forEach(function(n,ix) {
		var s = 0;
		E.forEach(function(e,n) {
		    var i = e[0];
		    if (ix == i) { // we only operate on edges related to i
			var j = e[1];
			var A = new THREE.Vector2(X[i*2],X[i*2+1]);
			var B = new THREE.Vector2(X[j*2],X[j*2+1]);
			var len = A.distanceTo(B);
			var q = 0;
			if (len < Y[n]) {
			    q += -2;
			}
			if (len > Z[n]) {
			    q += 2;
			}
			if (q) {
			    var temp = ((A.x - B.x) + (A.y - B.y));
			    s += q * temp;
			    v += temp*temp;
			}
		    }
		});
		d[ix] = s;
	    });
//	    console.log("v",v);
	    return [v,d];	    
	}

        var solution = fmin.conjugateGradient(f, fxprime, params);
	console.log(solution);
    });
    

});
