'use strict';
var assert = require('assert');

var ANO = require('../actnetopt.js');
var fmin = require('../javascripts/fmin.js');
var algols = require('algorithms').DataStructures;
var THREE = require('../javascripts/three.js');

console.log(fmin);

var SMALL = 1e-5;

var x = 1.6084564160555601, y = -1.5980748860165477;
function banana(X, fxprime) {
    fxprime = fxprime || [0, 0];
    var x = X[0], y = X[1];
    fxprime[0] = 400 * x * x * x - 400 * y * x + 2 * x - 2;
    fxprime[1] = 200 * y - 200 * x * x;
    return (1 - x) * (1 - x) + 100 * (y - x * x) * (y - x * x);
}


describe('fmin', function() {
    describe('banana function', function() {
	var params = {'learnRate' : 0.0003, 'maxIterations' : 50000};	
        var solution = fmin.conjugateGradient(banana, [x, y], params);
	console.log(solution);
    });

    describe('zero function', function() {
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


});
