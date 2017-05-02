'use strict';
var assert = require('assert');

var ano = require('../actnetopt.js');
var algols = require('algorithms').DataStructures;
var THREE = require('../javascripts/three.js');

Math.distance3 = function(p0,p1) {
    x = p0[0] - p1[0];
    y = p0[1] - p1[1];
    z = p0[2] - p1[2];        
    return Math.sqrt(x*x + y*y + z*z);
}

function simple_triangle_problem() {
    var dim = ano.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {}
	    };

    m.g.addVertex('a');
    m.g.addVertex('b');
    m.g.addVertex('c');
    m.g.addEdge('a','b');
    m.g.addEdge('b','c');
    m.g.addEdge('c','a');

    // ARG -- this need to be edge, not per node,
    // although really it is constant in most circumstances.
    m.lbs['a b'] = 1;
    m.ubs['a b'] = 2;
    m.lbs['b c'] = 1;
    m.ubs['b c'] = 2;
    m.lbs['a c'] = 1;
    m.ubs['a c'] = 2;
	    
    var fixed = [];
    fixed[0] = 'a';
    fixed[1] = 'b';

    var goals = [];
    goals[0] = { nd: 'c',
		 pos: new THREE.Vector2(4,4),
		 wt: 3 };

    var nodes = {};
    var a = new THREE.Vector2(0,0);
    var b = new THREE.Vector2(0,1.5);
    var c = new THREE.Vector2(1,1);
    nodes['a'] = a;
    nodes['b'] = b;
    nodes['c'] = c;
    
    return { dim: dim,
	     coords: nodes,
	     model: m,
	     goals: goals,
	     fixed: fixed};
}

function medium_triangle_problem() {
    var dim = ano.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {}
	    };

    m.g.addVertex('a');
    m.g.addVertex('b');
    m.g.addVertex('c');
    m.g.addVertex('d');    
    m.g.addEdge('a','b');
    m.g.addEdge('b','c');
    m.g.addEdge('c','a');
    m.g.addEdge('b','d');
    m.g.addEdge('c','d');

    // ARG -- this need to be edge, not per node,
    // although really it is constant in most circumstances.
    m.lbs['a b'] = 1;
    m.ubs['a b'] = 2;
    m.lbs['b c'] = 1;
    m.ubs['b c'] = 2;
    m.lbs['a c'] = 1;
    m.ubs['a c'] = 2;
    m.lbs['c d'] = 1;
    m.ubs['c d'] = 2;
    m.lbs['b d'] = 1;
    m.ubs['b d'] = 2;

    
    var fixed = [];
    fixed[0] = 'a';

    var goals = [];
    goals[0] = { nd: 'd',
		 pos: new THREE.Vector2(4,4),
		 wt: 3 };

    var nodes = {};
    var a = new THREE.Vector2(0,0);
    var b = new THREE.Vector2(0,1.5);
    var c = new THREE.Vector2(1,1);
    var d = new THREE.Vector2(0.25,2.5);    
    nodes['a'] = a;
    nodes['b'] = b;
    nodes['c'] = c;
    nodes['d'] = d;    
    
    return { dim: dim,
	     coords: nodes,
	     model: m,
	     goals: goals,
	     fixed: fixed};
}

describe('actoptnet_math', function() {
    /*
    describe('first_test', function() {
	it('has a variable that is true', function() {
	    assert.equal(ano.dim2,2);
	});
    });
    describe('main_algorithm', function() {
	it('function exists and returns something', function() {
	    var stm = simple_triangle_problem();

	    var result = ano.opt(stm.dim2,
				 stm.model,
				 stm.coords,
				 stm.goals,
				 stm.fixed);

	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);	    
	});
    });
    describe('main_algorithm', function() {
	it('we can do something with a 2d triangle', function() {
	    var dim = ano.dim2;
	    var model = new algols.Graph();
	    model.addVertex(1);
	    model.addVertex(2);
	    model.addVertex(3);
	    model.addEdge(1, 2);
	    model.addEdge(2, 3);
	    model.addEdge(1, 3);
	    assert.equal(model.vertices.size,3);
	});
    });
    describe('limits', function() {
	it('limits returns a useful answer', function() {
	    var stm = simple_triangle_problem();
	    var limits = ano.limits(stm.model,stm.coords,stm.goals[0]);
	    assert.equal(limits[0][0],'b');
	    assert.equal(limits[1][0],'c');
	    assert(limits[0][1]);
	    assert(limits[1][1]);	    
	});
    });
    describe('score', function() {
	it('we can compute a score', function() {
	    var stm = simple_triangle_problem();
	    var sc = ano.score(stm.coords, stm.goals);
	    assert.equal(sc,54);
	});
    });
    */
    describe('main_algorithm', function() {
	it('we can invoke it', function() {

	    var stm = simple_triangle_problem();

	    var result = ano.opt(stm.dim2,
				 stm.model,
				 stm.coords,
				 stm.goals,
				 stm.fixed);
	    
	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);
	});
    });
    describe('main_algorithm', function() {
	it('We get a good answer for a siimple triangle', function() {
	    var stm = simple_triangle_problem();

	    var result = ano.opt(stm.dim2,
				 stm.model,
				 stm.coords,
				 stm.goals,
				 stm.fixed);
	    var score = ano.score(result,stm.goals);
	    console.log(result,score);

	    console.log(stm.model);
	    assert.equal(ano.legal_configp(stm.model,result),true);

	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);
	    console.log(score);
	    assert.ok(score < 40.2);
	});
    });
    describe('main_algorithm', function() {
	it('We can deal with a more complex function', function() {
	    var stm = medium_triangle_problem();
	    assert.equal(ano.legal_configp(stm.model,stm.coords),true);
	    
	    var result = ano.opt(stm.dim2,
				 stm.model,
				 stm.coords,
				 stm.goals,
				 stm.fixed);
	    var score = ano.score(result,stm.goals);
	    console.log(result,score);

	    console.log(stm.model);
	    assert.equal(ano.legal_configp(stm.model,result),true);

	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);
	    assert(score < 20);
	});
    });
});
