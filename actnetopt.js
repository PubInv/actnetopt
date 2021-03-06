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

// This is an attempt to develop an algorithm for optimizing nodes within an
// actuator net. An actuator is abstractly a device that connects nodes
// and allows them to move between a minimum and maximum distance.


"use strict";

var algols = require('algorithms').DataStructures;
var heap = require('algorithms').DataStructures.Heap;
var dijkstra = require('algorithms').Graph.dijkstra;
var THREE = require('./javascripts/three.js');
var opt = require('./javascripts/optimization.js');
var algols = require('algorithms').DataStructures;

var assert = require('assert');

// Let's see if we can just do a good job computing
// a score from a model

module.exports.dim2 = 2;
module.exports.DEBUG_LEVEL = 1;

module.exports.opt = opt;

module.exports.score1 = function(coords,g) {
    var c = coords[g.nd];
    var q = g.pos.distanceToSquared(c);
    return q * g.wt;
}
module.exports.score = function(coords,goals) {
    var sum = goals.map(g =>  this.score1(coords,g)).reduce(
	function(acc,elem) {
	    return acc+elem;
	},0);
    return sum;
}


Math.distance3 = function(p0,p1) {
    x = p0[0] - p1[0];
    y = p0[1] - p1[1];
    z = p0[2] - p1[2];        
    return Math.sqrt(x*x + y*y + z*z);
}

function gen_nodeset(n) {
    const nodeset = [];
    for(var i = 0; i < n; i++) {
	nodeset.push(String.fromCharCode(97 + i));
    }
    return nodeset;
}
function ename(x,y) {
    return (x < y) ? x + ' ' + y
	: y + ' ' + x;
}

function  construct_bounds(m,ns,lb,ub) {
    for(var i in ns) {
	for(var j in ns) {
	    if (i != j) {
		const en = ename(ns[i],ns[j]);
		m.lbs[en] = m.deflb;
		m.ubs[en] = m.defub;
	    }
	}
    }
}
function gen_regular_2d_net(m,nodeset) {
    assert(nodeset.length >= 3);
    m.g.addEdge('a','b');
    m.g.addEdge('b','c');
    m.g.addEdge('c','a');
    for(var i = 3; i < nodeset.length; i++) {
	var p0 = nodeset[i-2];
	var p1 = nodeset[i-1];
	var c0 = nodeset[i];
	m.g.addEdge(p0,c0);
	m.g.addEdge(p1,c0);
    }
}


function gen_simple_3d_net(m,nodeset) {
    assert(nodeset.length >= 3);
    m.g.addEdge('a','b');
    m.g.addEdge('b','c');
    m.g.addEdge('c','a');
    m.g.addEdge('a','d');
    m.g.addEdge('b','d');
    m.g.addEdge('c','d');
    
}

module.exports.create_lengths_object = function(m,nodeset,init_len) {
    var d = {}; // d maps an edge name to an observed length...
    var standard_length = 1.3;
    nodeset.forEach(nda => {
	m.g.neighbors(nda).forEach(ndb => {
	    if (nda < ndb) {
		var en = ename(nda,ndb);
		d[en] = init_len;
	    }
	})
    }
		   );
    return d;
}
    

module.exports.create_standard_observation = function(n) {
    var dim = this.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {},
	      deflb: 1.1,
	      defub: 2,
	      fixed: {}
	    };

    const nodeset = gen_nodeset(n);
    nodeset.forEach(nd => m.g.addVertex(nd));

    construct_bounds(m,nodeset,m.deflb,m.defub);
    gen_regular_2d_net(m,nodeset);

    var fixed = {};
    fixed['a'] = true;
    fixed['b'] = true;
    m.fixed = fixed;

    // Because this is an observation, we don't have
    // goals and node coordinates, but rather observed
    // distantces, which I will call "d".

    var d = {}; // d maps an edge name to an observed length...
    var standard_length = 1.3;
    nodeset.forEach(nda =>
		    m.g.neighbors(nda).forEach(ndb => {
			if (nda < ndb) {
			    var en = ename(nda,ndb);
			    d[en] = standard_length;
			}
		    })
		   );

    var d = this.create_lengths_object(m,nodeset,standard_length);


// However, we need to have coordinates for the "fixed" nodes!
    // The first two nodes are on the y axis!
    var nodes = {};
    // in fact a and b are fixed, but we will still use this
    // as the initial position of our use of optimize.js
    nodeset.forEach((nd,ix) => nodes[nd]
		    = new THREE.Vector2(Math.floor(ix/2),
					((ix % 2) == 0) ? 0 : 1.6));
/*    var a = ;
    var b = new THREE.Vector2(0,1.5);
    var c = new THREE.Vector2(0.5,0.5);
    nodes['a'] = a;
    nodes['b'] = b;
    nodes['c'] = c;
*/
    
    return { dim: dim,
	     coords: nodes,
	     model: m,
	     nodeset: nodeset,	     
	     d: d,
//	     goals: goals,
	     fixed: fixed};
}

// MAJOR CONCEPTUAL PROBLEM: my algorithm is pretty much assuming
// that all the lowerbounds and upper bounds are the same.
module.exports.simple_triangle_problem = function() {
    var dim = this.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {},
	      deflb: 1.1,
	      defub: 2,
	      fixed: {}
	    };

    const nodeset = gen_nodeset(3);
    nodeset.forEach(nd => m.g.addVertex(nd));

    construct_bounds(m,nodeset,m.deflb,m.defub);
    gen_regular_2d_net(m,nodeset);

    var fixed = {};
    fixed['a'] = true;
    fixed['b'] = true;
    m.fixed = fixed;

    var goals = [];
    goals[0] = { nd: 'c',
		 pos: new THREE.Vector2(4,4),
		 //		 pos: new THREE.Vector2(9,6),
//		 pos: new THREE.Vector2(0,6),		 		 
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
	     nodeset: nodeset,
	     goals: goals,
	     fixed: fixed};
}


module.exports.simple_tetrahedron_problem = function() {
    var dim = this.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {},
	      deflb: 1.1,
	      defub: 2,
	      fixed: {}
	    };

    const nodeset = gen_nodeset(4);
    nodeset.forEach(nd => m.g.addVertex(nd));

    construct_bounds(m,nodeset,m.deflb,m.defub);
    
    gen_simple_3d_net(m,nodeset);

    var fixed = {};
    fixed['a'] = true;
    fixed['b'] = true;
    m.fixed = fixed;

    var goals = [];
    goals[0] = { nd: 'c',
		 pos: new THREE.Vector3(4,4),
		 wt: 3 };

    var nodes = {};
    var a = new THREE.Vector3(0,0,0);
    var b = new THREE.Vector3(0,1.5,0);
    var c = new THREE.Vector3(1,1,0);
    var d = new THREE.Vector3(0.5,0.5,1);    
    nodes['a'] = a;
    nodes['b'] = b;
    nodes['c'] = c;
    nodes['d'] = d;    
    
    return { dim: dim,
	     coords: nodes,
	     model: m,
	     nodeset: nodeset,	     
	     goals: goals,
	     fixed: fixed};
}

module.exports.medium_triangle_problem = function() {
    var dim = this.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {},
	      deflb: 1.1,
	      defub: 2,
	      fixed: {}
	    };
    const nodeset = gen_nodeset(4);
    nodeset.forEach(nd => m.g.addVertex(nd));    
    construct_bounds(m,nodeset,m.deflb,m.defub);
    gen_regular_2d_net(m,nodeset);
    
    var fixed = {};
    fixed['a'] = true;
    m.fixed = fixed;

    var goals = [];
    goals[0] = { nd: 'd',
		 pos: new THREE.Vector2(3,3),
		 wt: 3 };

    var nodes = {};
    var a = new THREE.Vector2(0,0);
    var b = new THREE.Vector2(0,1.5);
    var c = new THREE.Vector2(1,1);
    var d = new THREE.Vector2(1.5,2);    
    nodes['a'] = a;
    nodes['b'] = b;
    nodes['c'] = c;
    nodes['d'] = d;    
    
    return { dim: dim,
	     coords: nodes,
	     model: m,
	     nodeset: nodeset,	     
	     goals: goals,
	     fixed: fixed};
}

function gen_regular_2d_coords(m,nodeset) {
    var med = (m.deflb + m.defub)/2;
    var h = (Math.sqrt(3))/2* med;
    var nodes = {};
    
    for(var i = 0; i < nodeset.length; i++) {
	var nd = nodeset[i];
	var x = (i % 2) * h;
	var y = i*(med/2)
	var p = new THREE.Vector2(x,y);
//	console.log("NODE",i,nd,x,y,p);	
	nodes[nd] = p;
    }
    return nodes;
}

module.exports.medium_triangle_problem2 = function() {
    var dim = this.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {},
	      deflb: 1.1,
	      defub: 2,
	      fixed: {}
	    };
    const nodeset = gen_nodeset(5);
    nodeset.forEach(nd => m.g.addVertex(nd));
    gen_regular_2d_net(m,nodeset);


    construct_bounds(m,nodeset,m.deflb,m.defub);
    var fixed = {};
    fixed['a'] = true;
    m.fixed = fixed;

    var goals = [];
    goals[0] = { nd: 'e',
		 pos: new THREE.Vector2(3.9,-2),
		 wt: 3 };

    const nodes = gen_regular_2d_coords(m,nodeset);


    console.log("NODES",nodes);
    
    return { dim: dim,
	     coords: nodes,
	     model: m,
	     nodeset: nodeset,	     
	     goals: goals,
	     fixed: fixed};
}

module.exports.big_triangle_problem = function() {
    var dim = this.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {},
	      deflb: 1.1,
	      defub: 2,
	      fixed: {}
	    };
    const nodeset = gen_nodeset(15);
    nodeset.forEach(nd => m.g.addVertex(nd));
    gen_regular_2d_net(m,nodeset);


    construct_bounds(m,nodeset,m.deflb,m.defub);
    var fixed = {};
    fixed['a'] = true;
    m.fixed = fixed;

    var goals = [];
    goals[0] = { nd: 'o',
		 pos: new THREE.Vector2(7,7),
		 wt: 3 };

    const nodes = gen_regular_2d_coords(m,nodeset);


//    console.log("NODES",nodes);
    
    return { dim: dim,
	     coords: nodes,
	     model: m,
	     nodeset: nodeset,	     
	     goals: goals,
	     fixed: fixed};
}


// This is from Algorithm.js ...
// I am forced to include here because it doesn't provide
// a way for me to use my own comparator in the PrioirtyQueue.
// I need to turn this into a pull request, and push this code back down into algorithm.js
// var MinHeap = require('./heap').MinHeap;

/**
 * Extends the MinHeap with the only difference that
 * the heap operations are performed based on the priority of the element
 * and not on the element itself
 */
function PriorityQueue(comp,initialItems) {
  var self = this;
  heap.MinHeap.call(this, function(a, b) {
      return comp(self.priority(a),self.priority(b)) < 0 ? -1 : 1;
  });

  this._priority = {};

  initialItems = initialItems || {};
  Object.keys(initialItems).forEach(function(item) {
    self.insert(item, initialItems[item]);
  });
}

PriorityQueue.prototype = new heap.MinHeap();

PriorityQueue.prototype.insert = function(item, priority) {
    if (this._priority[item] !== undefined) {
	console.log("treating as change!");
    return this.changePriority(item, priority);
  }
  this._priority[item] = priority;
  heap.MinHeap.prototype.insert.call(this, item);
};

PriorityQueue.prototype.extract = function(withPriority) {
  var min = heap.MinHeap.prototype.extract.call(this);
  return withPriority ?
    min && {item: min, priority: this._priority[min]} :
    min;
};

PriorityQueue.prototype.priority = function(item) {
  return this._priority[item];
};

PriorityQueue.prototype.changePriority = function(item, priority) {
  this._priority[item] = priority;
  this.heapify();
};

//module.exports = PriorityQueue;

// For the goal cg, compute the nodes that limit
// our ability to reach cg, returning the list
// of potentially movable (not-fixed) nodes.
// if the there are not limits, it returns "nolimits"
// if there are limits but no nodes that can be moved,
// it returns the empty list.
module.exports.limits = function(model,cur,cg) {
    var nam = cg.nd;
    var pos = cur[nam];
    // TODO: This is dim-dependent.
    var dir = this.copy_vector(cg.pos);
    dir.subVectors(dir,pos);

    var limits = [];
    // now we need to iterate over only the connected nodes from model...
    var neighbors = model.g.neighbors(nam);
    var nolimits = true;
    neighbors.forEach(neighbor => {
	var ename = this.ename(nam,neighbor);
	var lb = model.lbs[ename];
	var ub = model.ubs[ename];
	var npos = cur[neighbor];
	
	var d = npos.distanceTo(cg.pos);
	
	var ndir = this.copy_vector(cg.pos);
	ndir.sub(npos);

	// TODO: this should be replacable
	// Note: it is not clearly picking lowest move distance is
	// really the right metric here. Is new_goal even guaranteed to be legal?
	// Only the path between npos and new_goal hits no other constraint.
	// So that does make it seem like the shortest move is best.
	
	if (d < lb) {
	    var delta = lb - d;
	    var new_goal = this.copy_vector(npos);
	    var s = lb / ndir.length();
	    
	    new_goal.addScaledVector(dir,s);

	    var dtm = new_goal.distanceTo(pos);
	    limits.push([neighbor,dtm,new_goal]);
	    nolimits = false;
	} else if (d > ub) {
	    var delta = d - ub;
	    var new_goal = this.copy_vector(npos);
	    ndir.setLength(ub);
	    new_goal.add(ndir)
	    var dtm = new_goal.distanceTo(pos);
	    limits.push([neighbor,dtm,new_goal]);
	    nolimits = false;
	} else {
	    // we're okay!
	}
    });

    return (nolimits) ? "nolimits" : limits;
}


// Okay, now limits is a set of nodes and new goal positions for them.
// However, we only want move the node up to the smallest limit.
// if the limits tells us to move in opposit directions, we are in some trouble!
// We want to return the position to move to.
module.exports.min_move = function(limits,pos) {
    // find the position that is definitely inside the limits!
    var init = limits[0];
    var possible_move = limits.reduce(function(acc,val) {
	return (Math.abs(val[1]) < Math.abs(acc[1])) ?
	    val : acc;
    },
				      init);
    return possible_move;
}

module.exports.copy_vector = function(v) {
    var nv =  (v instanceof THREE.Vector2) ?
		    new THREE.Vector2(v.x,v.y) :
	new THREE.Vector3(v.x,v.y,v.z);
    return nv;
}

module.exports.legal_configp = function(model,config) {
    var string = "";
    var epsilon = 0.00000000000001;
    model.g.vertices.forEach(
	v0 =>
	    model.g.neighbors(v0).forEach(
		v1 =>
		    {
			var c0 = config[v0];
			var c1 = config[v1];
			var d = c0.distanceTo(c1);
			var ename = (v0 < v1) ? v0 + ' ' + v1
			    : v1 + ' ' + v0;
			if (d < (model.lbs[ename] - epsilon)) {
			    string += "lower bound not met: " + ename + " " + d + " \n";
			    return ename;
			}
			if (d > (model.ubs[ename] + epsilon)) {
			    string += "upper bound not met: " + ename +  " " + d  + " \n";
			    return ename;
			}
		    }
	    )
    );
    return (string == "") ? true : string;
}

module.exports.max_non_compliant = function(model,config) {
    var string = "";
    var epsilon = 0.00000000000001;
    var max_non_compliance = 0;
    model.g.vertices.forEach(
	v0 =>
	    model.g.neighbors(v0).forEach(
		v1 =>
		    {
			var c0 = config[v0];
			var c1 = config[v1];
			var d = c0.distanceTo(c1);
			var ename = (v0 < v1) ? v0 + ' ' + v1
			    : v1 + ' ' + v0;
			if (d < model.lbs[ename]) {
			    max_non_compliance = Math.max(max_non_compliance,
							  Math.abs(model.lbs[ename] - d));
			    string += "lower bound not met: " + ename + " " + d + " \n";
			}
			if (d > model.ubs[ename]) {
			    string += "upper bound not met: " + ename +  " " + d  + " \n";
			    max_non_compliance = Math.max(max_non_compliance,
							  Math.abs(model.ubs[ename] - d));
			}
		    }
	    )
    );
    console.log("NON_COMPLIANCE",string);
    return max_non_compliance;
}

module.exports.max_non_compliant_debug = function(model,config) {
    var string = "";
    var epsilon = 0.00000000000001;
    var max_non_compliance = 0;
    string += "\n";
    model.g.vertices.forEach(
	v0 =>
	    model.g.neighbors(v0).forEach(
		v1 =>
		    {
			if (v0 < v1) {
			var c0 = config[v0];
			var c1 = config[v1];
			var d = c0.distanceTo(c1);
			var ename = (v0 < v1) ? v0 + ' ' + v1
			    : v1 + ' ' + v0;
			    string += "distance: " + ename + " " + d + " \n";
			}
		    }
	    )
    );
    string += "\n";
    console.log("NON_COMPLIANCE",string);
    return max_non_compliance;
}


// I believe I need an array comparison to implement my own heap.
// basically I want to just to lexicographic sorting of an array in
// order of the elements, which should be a one-liner(ish).
// The main order that matters to me for the priority queue is:
// Distance from a goal,
// Weight of the goal
function compare_arr(arra,arrb) {
    for(var i = 0; i < arra.length; i++) {
	if (arra[i] < arrb[i])
	    return -1;
	if (arra[i] > arrb[i])
	    return 1;
    }
    return 0;
}

module.exports.ename = ename;

// d is the dimension (2 or 3)
// M is the connectivity graph
// C is the current coordinates of the nodes
// x is the tail of the strainfront vector
// y is the head of the strainfront vector
// return 0 if there is no strain, negative number
// if there is tensile strain, positive if compressive strain.
module.exports.strain = function(d,M,C,x,y)  {
    var px = C[x], py = C[y];
    return this.strain_points(d,M,x,y,px,py);
}

module.exports.strain_points = function(d,M,x,y,px,py)  {
//    console.log("STRAIN_POINTS x,y,px,y",x,y,px,py);
    var b = px.distanceTo(py);
    
    var en = this.ename(x,y);
///	console.log("STRAIN_POINTS ",M.lbs);	    
    if (en in M.lbs) {

	var lb = M.lbs[en];
//	console.log("STRAIN_POINTS lb",lb);	
	if (lb > b) {
//	    console.log("STRAIN_POINTS lb- b",x,y,lb-b);
	    return lb - b;
	}
    }
//    console.log("STRAIN_POINTS ",M.ubs);		    
    if (en in M.ubs) {

	var ub = M.ubs[en];
//	console.log("STRAIN_POINTS ub",ub);
	if (ub < b) {
	    // tensile strain is negative
//	    console.log("STRAIN_POINTS ub - b",x,y,ub-b);	    
	    return ub - b;
	}
    }
    return 0;
}

// This code from jupdike:    
// https://gist.github.com/jupdike/bfe5eb23d1c395d8a0a1a4ddd94882ac   

// based on the math here:
// http://math.stackexchange.com/a/1367732
    
// x1,y1 is the center of the first circle, with radius r1
// x2,y2 is the center of the second ricle, with radius r2
module.exports.intersectTwoCircles = function (x1,y1,r1, x2,y2,r2) {
//    console.log("COMPS == ",x1,y1,r1,x2,y2,r2);        
  var centerdx = x1 - x2;
  var centerdy = y1 - y2;
  var R = Math.sqrt(centerdx * centerdx + centerdy * centerdy);
  if (!(Math.abs(r1 - r2) <= R && R <= r1 + r2)) { // no intersection
    return []; // empty list of results
  }
  // intersection(s) should exist

  var R2 = R*R;
  var R4 = R2*R2;
  var a = (r1*r1 - r2*r2) / (2 * R2);
  var r2r2 = (r1*r1 - r2*r2);
  var c = Math.sqrt(2 * (r1*r1 + r2*r2) / R2 - (r2r2 * r2r2) / R4 - 1);

  var fx = (x1+x2) / 2 + a * (x2 - x1);
  var gx = c * (y2 - y1) / 2;
  var ix1 = fx + gx;
  var ix2 = fx - gx;

  var fy = (y1+y2) / 2 + a * (y2 - y1);
  var gy = c * (x1 - x2) / 2;
  var iy1 = fy + gy;
  var iy2 = fy - gy;

  // note if gy == 0 and gx == 0 then the circles are tangent and there is only one solution
  // but that one solution will just be duplicated as the code is currently written
  return [[ix1, iy1], [ix2, iy2]];
}

// return an array of circle intersections (0,1, or 2)
// This only works with 2 dimensions right now!!
module.exports.circle_intersections = function(v0,r0,v1,r1,tag) {
    var x0 = v0.x;
    var x1 = v1.x;
    var y0 = v0.y;
    var y1 = v1.y;
//    console.log("FROM cirlce_intersections COMPS == ",x0,y0,r0,x1,y1,r1);    
    var is = this.intersectTwoCircles(x0,y0,r0,x1,y1,r1);
//    console.log("IS == ",is);
    if ((is.length == 2)
	&& (is[0][0] == is[1][0])
	&& (is[0][1] == is[1][1])) {
	is = is.slice(0,1);
    }
//    console.log("IS == ",is);    
    return is.map(ip => {
//	console.log("IP == ",ip);	
	return { tag: tag, p: new THREE.Vector2(ip[0],ip[1])};
    });
}

function isNumeric(n) {
  return !isNaN(parseFloat(n)) && isFinite(n);
}
// d is the dimension (2 or 3)
// M is the connectivity graph
// C is the current coordinates of the nodes
// a is a node connected to b
// b is a node connected to a
// return the labelled intersectiosn of the boundaries of a,b
module.exports.bound_intersections = function(d,M,C,a,b) {
    assert(a != b,a);
    var en = this.ename(a,b);
    var lb,ub;

    var ubis = [];
    var luis = [];
    var ulis = [];
    var lbis = [];
    
//    if (en in M.lbs) {
	lb = M.lbs[en]  || M.deflb;
	lbis = this.circle_intersections(C[a],lb,C[b],lb,
					 { b0: 'lb', nd0: a, b1: 'lb', nd1: b});
//    }
//    if (en in M.ubs) {
	ub = M.ubs[en]  || M.defub;
	ubis = this.circle_intersections(C[a],ub,C[b],ub,
					 { b0: 'ub', nd0: a, b1: 'ub', nd1: b});
//	console.log("UBIS",a,b,ub,ubis);
//    }
//    if (isNumeric(ub) && isNumeric(lb)) {
	luis = this.circle_intersections(C[a],lb,C[b],ub,
					 { b0: 'lb', nd0: a, b1: 'ub', nd1: b});
	ulis = this.circle_intersections(C[a],ub,C[b],lb,
					{ b0: 'ub', nd0: a, b1: 'lb', nd1: b});
//    }
//    console.log("LB, UB", lb,ub,M.deflb,M.defub);
    var is = [];
    var is = is.concat(ubis).concat(luis).
	concat(ulis).concat(lbis);
    
    return is;
}

// d is the dimension (2 or 3)
// M is the connectivity graph
// C is the current coordinates of the nodes
// x is the node name
// p is the candidate position of x
module.exports.all_strains = function(d,M,C,x,p) {
    return M.g.neighbors(x).map( y =>
				 {
				     var s = this.strain_points(d,
								M,
								x,
								y,
								p,
								C[y]);
//				     console.log("S =",x,p,y,C[y],s);
				     return s;
				  }
				 );
}

// Compute the maximum strain on node x if it is at point p.
module.exports.max_strain_on_point = function(d,M,C,x,p) {
    const as = this.all_strains(d,M,C,x,p);
    
    var max_strain = as.reduce((a,v) =>
			       Math.max(Math.abs(v),a)
			       ,0);
    return max_strain;
}

// d is the dimension (2 or 3)
// M is the connectivity graph
// C is the current coordinates of the nodes
// S is the current strainfront
// x is the tail of the strainfront vector
// y is the head of the strainfront vector
// return z, the position of y which completely eases x->y strain.
// This needs to be rewritten to use x and y more effectively.
// We cannot be constrained to intersections only.  Instead, we really want
// to dry a straight line which is still a legal line.
// I think we can use optimize.js to find a legal point in this limited space.
module.exports.zero_x_strain = function(d,M,C,x,y) {
//    console.log("x :",x, "y :", y, "C[x] :",C[x]);
//    console.log("C :",C);    
    // first, let us determine if there is any strain.
    var s = this.strain(d,M,C,x,y);
    var retval; 
    if (s == 0) {
	// No change to C required.
	retval =  C[y];
    } else {
	// Now we want to find the set of points representing
	// the intersection points of all bounds for all neighbors of y
	var intersections = [];
	M.g.neighbors(y).forEach(
	    v0 =>
		{
		    M.g.neighbors(y).forEach(
			v1 => {
			    if (v0 == v1) return; // do nothing
			    else { // compute v0 v1 intersections (0,1, or 2, and add)
				if (v0 < v1) {
				    var en = this.ename(v0,v1);
//				    console.log("ENAME",en);
				    var ints = this.bound_intersections(d,M,C,v0,v1);
//				    console.log("intersections",C[v0],C[v1],ints);				    
				    intersections = intersections.concat(ints);
				}
			    }
			});
		}
	);
//	console.log("BOUND INTERSECTIONS = ",intersections);
	// now that we have the intersections, we want to see if there is one
	// that has no strain at all...
	// so we iterate of all intersections, seeking a point that
	// has no strains...
	var zero_strain_point;
	intersections.forEach( i => {
	    const py = i.p;
	    var max_strain = this.max_strain_on_point(d,M,C,y,py);
//	    console.log("QQQ",this.all_strains(d,M,C,y,py));	    
//	    console.log("ZZZ",y,py,max_strain);
	    if (max_strain == 0) {
//		console.log("AAA : ",i.p,i.p.distanceTo(C[y]));

		// My animation of this algorithm now leads me to believe this is making
		// a bad choice.
		if (!zero_strain_point ||
		    i.p.distanceTo(C[y]) < zero_strain_point.distanceTo(C[y])) {
		    zero_strain_point = i.p;
//		    console.log("BBB: ",zero_strain_point,zero_strain_point.distanceTo(C[y]));
		}
	    }
	});
//	if (zero_strain_point)
//	    console.log("END: ",zero_strain_point,zero_strain_point.distanceTo(C[x]));		
	// if we have found a zero_strain_point, we should surely return that!
	// Note: This is type dependent
	if (zero_strain_point instanceof Object) {
//	    console.log("zero strain point",zero_strain_point);
	    retval = zero_strain_point;
	} else {
	    // Since we have no intersection point which is universally strain-free,
	    // let's at least see if we have a point that x-strain-free, and choose
	    // between those.
	    var zero_x_strain_points =
		intersections.reduce( (acc,i) => {
		    var xs =
			this.strain_points(d,
					   M,
					   x,
					   y,
					   C[x],
					   i.p);
		    if (xs == 0) {
			acc.push(i.p);
			return acc;
		    } else {
			return acc;
		    }
		},
				      []);
//	    console.log("zero_x_strain: ",zero_x_strain_points);
	    if (zero_x_strain_points.length != 0) { // we're in luck...
		// we can should return one which is closest to C[x]...
		retval = zero_x_strain_points.reduce(
		    (acc,p) =>
			(p.distanceTo(C[y]) < acc[0]) ?
			[p.distanceTo(C[y]),p] :
			acc,
		    [Number.MAX_VALUE,zero_x_strain_points[0]])[1];
	    } else {
//		console.log("NO ZERO_X_STRAIN POINTS!");
		// Okay, so if there are no points of zero x strain,
		// we will have to choose a point on a line that is NOT an
		// intersection point.
		// One idea is to draw a line to the centroid of the free area,
		// or to the closest point to y in in the free area of y (assuming that
		// we don't count x). Then try to get
		// NOTE: I think now that maybe this is the place for Powell's method
		var g = this.copy_vector(C[y]);
		g.sub(C[x]);
		// Now g is a vector from C[x] to C[y].
		var en = this.ename(x,y);
//		console.log("g = ",g);
		g = g.setLength((s < 0) ? M.ubs[en] : M.lbs[en]);
		g = g.add(C[x]);
		retval = g;
//		console.log("bound",(s < 0) ? M.ubs[en] : M.lbs[en]);				
//		console.log(C[x],C[y],g);		
//		console.log(this.strain_points(d,M,x,y,C[x],retval));
	    }
	}
    }
    //    console.log("retval",retval);

    // note: this is failing in some cases!
    if (this.DEBUG_LEVEL > 0)
	assert(
	    this.strain_points(d,M,x,y,C[x],retval) < SMALL,
	    `${x} = (${C[x].toArray()}), ${y} = (${C[y].toArray()}) `);
    
    return retval;
}

// d is the dimension (2 or 3)
// M is the connectivity graph
// C is the current coordinates of the nodes
// S is the current strainfront
// returns a strainfront
module.exports.perturb = function(d,M,C,S,a,v,num) {

    assert(!(a in M.fixed));
    assert(!(a in S.fixed));
    
    // if we have already perturbed this, we will not
    // perturb again unless on the backwave...
    var previously_perturbed = S.perturbed[a];
//    if (a in S.perturbed) {
//	return S;
    //    }
//    console.log("perturb:",a,v);
    S.cur[a] = v;
    S.perturbed[a] = true;
    if (!previously_perturbed) {
	M.g.neighbors(a)
	    .forEach(v0 => {
		// This is failing to catch us somehow
		if ((!S.s.find(sv => (sv.tl == a && sv.hd == v0)))
		    &&
		    (!S.used.find(sv => (sv.tl == a && sv.hd == v0)))) {
 		    S.s.push({tl:a, hd:v0, num: num});
//		    console.log("ADDING: ",{tl:a, hd:v0, num: num});
		}
		if ((!S.s.find(sv => (sv.tl == v0 && sv.hd == a)))
		    &&
		    (!S.used.find(sv => (sv.tl == v0 && sv.hd == a)))) {
		    S.s.push({tl:v0, hd:a,  num: -num});
//		    console.log("ADDING: ",{tl:v0, hd:a,  num: -num});		    
		}
	    });
    }
    return S;
}

// d is the dimension (2 or 3)
// M is the connectivity graph
// C is the current coordinates of the nodes
// S is the current strainfront
// This function is used for the back-propagating wave
// returns a strainfront
module.exports.reperturb = function(d,M,C,S,a,v,num) {
//    console.log("calling reperturb: ", a);
    S.cur[a] = v;
    return S;
}

module.exports.relieves = function(d,M,C,S,num) {
    // choose an element of the strainfront.
    // our algorithm is to choose the loosest numbered positive edge
    // non-negaive edge exists. (zero not allowed.)
    // If no positive edge, we choose the lowest-valued negative edge
    var lowest_non_negative = Number.MAX_VALUE;
    var lnn;
    var lowest_negative = Number.MAX_VALUE;
    var ln;
    for(var i = 0; i < S.s.length; i++) {
	var k = S.s[i];
	assert(k.num != 0);

	if ((k.num > 0) && (k.num < lowest_non_negative)) {
	    lnn = k;
	    lowest_non_negative = k.num;
	}
	if ((k.num < 0) && (k.num < lowest_negative)) {
	    ln = k;
	    lowest_negative = k.num;	    
	}
    }
    const v = (lnn instanceof Object)
	  ? lnn : ln;
    assert(v);
//    console.log("v : ",v.tl,v.hd, v.num);    
    // remove element v...
    S.s = S.s.filter(item => item !== v)
    S.used.push(v);
    // now v is a node of minum depth...
    if ((v.hd in M.fixed) || (v.hd in S.fixed)) {
	S.fixed[v.hd] = true;
	return S;
    } else {
	var z = this.zero_x_strain(d,M,S.cur,v.tl,v.hd);
//	console.log("in relieves",v,z);
	return  (v.num > 0) ?
	    this.perturb(d,M,C,S,v.hd,z,num)
		    : this.reperturb(d,M,C,S,v.hd,z,num)
	;
    }
    return S;
}

// d is the dimension (2 or 3)
// M is the connectivity graph
// C is the current coordinates of the nodes
// S is the current strainfront
// a the node to move
// v the desired position of a
// anim is an "animation" function callback to be called step
// through the algorithm.
// returns a configuration mapping nodes to points which
// is guaranteed to be legal. Hopefully a as close to v as possible.
module.exports.strainfront = function(d,M,C,a,v,anim) {
//    console.log("a =",a,M.fixed);
    assert(!(a in M.fixed),a)
    // I think we want to copy the initial configuration here into cur...
    const cur = {};
    Object.keys(C).forEach( nd => cur[nd] = this.copy_vector(C[nd]));
//    console.log(dijkstra);    
    var shortestPath = dijkstra(M.g, a);
//    console.log(shortestPath);
    var S = { s: [], used: [], cur: cur, fixed: {}, perturbed: {} , dsp: shortestPath};
    var num = 1;
    if (anim)
	if (!anim(S)) return S.cur;

    
    S = this.perturb(d,M,S.cur,S,a,v,num);

    if (anim)
	if (!anim(S)) return S.cur;
    
    num++;
    while (S.s.length > 0) {
//	console.log("pre-relieve",S.cur);
	S = this.relieves(d,M,S.cur,S,num);
//	console.log("post-relieve",S.cur);
	if (anim)
	    if (!anim(S)) return S.cur;
	num++;
    }
//    console.log("XXXX");
//    console.log(S);
    return S.cur;
}

module.exports.dijkstra = dijkstra;

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
var LINEAR_PENALTY_WT = 1;
    
module.exports.f = function(X,fxprime,stm,Y,Z,names,V,F) {
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
//    console.log("f,fxprime",rv[0] + pv[0],fxprime);
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
//    console.log("X",X);
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
//    console.log("X,initial",X,initial);
    assert(initial.length == X.length);
    assert((free+goal+fixed) == Object.keys(stm.coords).length);
    assert((F.length + X.length) == 2*Object.keys(stm.coords).length);	

    var Y = stm.model.deflb * stm.model.deflb;
    var Z = stm.model.defub * stm.model.defub;
    return [F,V,X,Y,Z,initial,names,fixed,goal,free];
}

module.exports.construct_optimization_model_from_ANO = construct_optimization_model_from_ANO;
module.exports.spud = 4;
