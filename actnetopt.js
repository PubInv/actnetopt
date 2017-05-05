
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
var assert = require('assert');

// Let's see if we can just do a good job computing
// a score from a model

module.exports.dim2 = 2;
module.exports.DEBUG_LEVEL = 1;

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

module.exports.simple_triangle_problem = function() {
    var dim = this.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {},
	      fixed: {}
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
	    
    var fixed = {};
    fixed['a'] = true;
    fixed['b'] = true;
    m.fixed = fixed;

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

module.exports.medium_triangle_problem = function() {
    var dim = this.dim2;
    var m = { g: new algols.Graph(false),
	      lbs: {},
	      ubs: {},
	      fixed: {}
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

    
    var fixed = {};
    fixed['a'] = true;
    m.fixed = fixed;

    var goals = [];
    goals[0] = { nd: 'd',
		 pos: new THREE.Vector2(4,4),
		 wt: 3 };

    var nodes = {};
    var a = new THREE.Vector2(0,0);
    var b = new THREE.Vector2(0,1.5);
    var c = new THREE.Vector2(1,1);
    var d = new THREE.Vector2(2,2);    
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
	var ename = (nam < neighbor) ? nam + ' ' + neighbor
	    : neighbor + ' ' + nam;
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
			    string += "lower bound not met: " + ename + " " + d + " \n";
			    return ename;
			}
			if (d > model.ubs[ename]) {
			    string += "upper bound not met: " + ename +  " " + d  + " \n";
			    return ename;
			}
		    }
	    )
    );
    return (string == "") ? true : string;
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

function find_goals(goals,name) {
    goals.find(g => g.nd == name);
}

module.exports.opt = function(dim,model,coords,goals) {
    // create my own set of goals in a priority queue...
    // The priority queue from algorithms.js is minimizing queue,
    // so we will use the negation of our scores, remembering
    // this.  We are in fact seeking a zero score.

    var cur = {};
    Object.keys(coords).forEach(
	c =>
	    {
		cur[c] =  this.copy_vector(coords[c]);
	    });
    var pq = new algols.PriorityQueue(compare_arr);

    // now it is possible that this can add a
    // zero goal...we will have to be careful of that.
    // Really I need to have my own comparator here that
    // orders by greated weighted distance first, then
    // by shortest move.
    goals.forEach(g =>
		  {
		      pq.insert(g.nd, [0,-this.score1(cur,g)])
		  });

    // xgoals is the set of goals we are currently working on,
    // which is a superset of the goals.
    var xgoals = goals.slice();
    
    var cnt = 0; 

    // now begin the interative processing...
    // we extract with priority so that we can implement breadth-first
    // effectively
    var cext = pq.extract(true);
    var cg = xgoals.find(g => g.nd == cext.item);

    while (cg && cnt < 3) {
	// Now cg is the "worst" goal we need to try to move...
	// compute the direction to move...
	var nam = cg.nd;
	var pos = cur[nam];
	var dir = this.copy_vector(pos);
	dir.subVectors(dir,pos)
	// Now we want to try to move in this direction until
	// we hit a constaint..
	// Possibly we should reorganize to put the limiting
	// nodes in the priority queue.
	var limiting_nodes = this.limits(model,cur,cg,dir);


	if (limiting_nodes != "nolimits") {
	    // Now compute the maximum move the limits allow
	    // (with 1.0 in case of empty...

	    if (limiting_nodes.length > 0) {
		var min_move = this.min_move(limiting_nodes,pos);

		// now min_move is a triple  chosen from limits....

		if (min_move[1] == 0.0) {
		    // In this case, there is no point n moving, we must move one of
		    // our neighbors (this one) in an attempt to get closer if we can.
		    // We need to add to our priority queue the limiting node as
		    // a goal, (and some new goal point).
		    var depth = cext.priority[0] + 1;
		    var a = min_move[0];
		    var apos = this.copy_vector(cur[a]);
		    var ldir = this.copy_vector(pos);
		    ldir.sub(apos);
		    var ename = (nam < a) ? nam + ' ' + a
			: a + ' ' + nam;
		    var median = (model.ubs[ename] - model.lbs[ename])/2.0;
		    var len = apos.distanceTo(cg.pos) - median;
		    ldir.setLength(len);
		    apos.add(ldir);
		    xgoals.push({ nd: a, pos: apos, wt: cg.wt });
		    // I'm pretty sure this is not really the right score to add.
		    var score = -10;
		    pq.insert(a,[depth,score]);
		} else {

		    // make the move...
		    cur[nam] = this.copy_vector(min_move[2]);
		    // rebuild the pq...
		    pq = new algols.PriorityQueue();

		    goals.forEach(g => pq.insert(g.nd, [0,-this.score1(cur,g)]));	    
		    // if we moved, then basically we start all over with all
		    // computations...
		}
	    } else {
		// we can't move, and every neighbor is fixed, so there
		// is nothing for us to put back in the queue.
	    }
	} else {
	    // If there are not limits, we can move directly to the goal...
	    // and we need add nothing to the pqueue...
	    cur[nam] = this.copy_vector(cg.pos);
	    pq = new algols.PriorityQueue();
	    goals.forEach(g => pq.insert(g.nd, [0,-this.score1(cur,g)]));	    
	}
	var cext = pq.extract(true);
	if (cext) {
	    cg = goals.find(g => g.nd == cext.item);
	} else {
	    cg = null;
	}
	cnt++;
    }
    return cur;
}

module.exports.ename = function(x,y) {
    return (x < y) ? x + ' ' + y
	: y + ' ' + x;
}

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
    
    if (en in M.lbs) {
	var lb = M.lbs[en];
	if (lb > b) {
//	    console.log("STRAIN_POINTS lb- b",lb-b);
	    return lb - b;
	}
    }
    if (en in M.ubs) {
	var ub = M.ubs[en];
	if (ub < b) {
	    // tensile strain is negative
//	    console.log("STRAIN_POINTS ub - b",ub-b);	    
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
function intersectTwoCircles(x1,y1,r1, x2,y2,r2) {
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
    var is = intersectTwoCircles(x0,y0,r0,x1,y1,r1);
    if ((is.length == 2)
	&& is[0][0] == is[1][0]
	&& is[0][1] == is[1][1]) {
	is = is.slice(0,1);
    }
    return is.map(ip => {
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
    
    if (en in M.lbs) {
	lb = M.lbs[en];
	lbis = this.circle_intersections(C[a],lb,C[b],lb,
					 { b0: 'lb', nd0: a, b1: 'lb', nd1: b});
    }
    if (en in M.ubs) {
	ub = M.ubs[en];
	ubis = this.circle_intersections(C[a],ub,C[b],ub,
					 { b0: 'lb', nd0: a, b1: 'lb', nd1: b});	
    }
    if (isNumeric(ub) && isNumeric(lb)) {
	luis = this.circle_intersections(C[a],lb,C[b],ub,
					 { b0: 'lb', nd0: a, b1: 'ub', nd1: b});
	ulis = this.circle_intersections(C[a],ub,C[b],lb,
					{ b0: 'ub', nd0: a, b1: 'lb', nd1: b});
    }
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
	this.strain_points(d,
			   M,
			   x,
			   y,
			   p,
			   C[y]));
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
				var en = this.ename(v0,v1);
				intersections = intersections.concat(this.bound_intersections(d,M,C,v0,v1));
			    }
			});
		}
	);
	// now that we have the intersections, we want to see if there is one
	// that has no strain at all...
	// so we iterate of all intersections, seeking a point that
	// has no strains...
	var zero_strain_point;
	intersections.forEach( i => {
	    const py = i.p;
	    var max_strain = this.max_strain_on_point(d,M,C,y,py);
//	    console.log("ZZZ",y,py,max_strain);
	    if (max_strain == 0) {
//		console.log("AAA : ",i.p,i.p.distanceTo(C[y]));
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
			(p.distanceTo(C[x]) < acc[0]) ?
			[p.distanceTo(C[x]),p] :
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
    if (this.DEBUG_LEVEL > 0)
	assert(
	    this.strain_points(d,M,x,y,C[x],retval) == 0,
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
    console.log("perturb:",a,v);
    S.cur[a] = v;
    S.perturbed[a] = true;
    if (!previously_perturbed) {
	M.g.neighbors(a)
	    .forEach(v0 => {
		if (!S.s.find(sv => (sv.tl == a && sv.hd == v0)))
		    S.s.push({tl:a, hd:v0, fwd: true, num: num});
		if (!S.s.find(sv => (sv.tl == v0 && sv.hd == a)))			      
		    S.s.push({tl:v0, hd:a, fwd: true, num: -num});			      
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
    console.log("calling reperturb: ", a);
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
    console.log("v : ",v);    
    // remove element v...
    S.s = S.s.filter(item => item !== v)
    // now v is a node of minum depth...
    if ((v.hd in M.fixed) || (v.hd in S.fixed)) {
	S.fixed[v.hd] = true;
	return S;
    } else {
	var z = this.zero_x_strain(d,M,S.cur,v.tl,v.hd);
	console.log("in relieves",v,z);
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
// returns a configuration mapping nodes to points which
// is guaranteed to be legal. Hopefully a as close to v as possible.
module.exports.strainfront = function(d,M,C,a,v) {
//    console.log("a =",a,M.fixed);
    assert(!(a in M.fixed),a)
    // I think we want to copy the initial configuration here into cur...
    const cur = {};
    Object.keys(C).forEach( nd => cur[nd] = this.copy_vector(C[nd]));
    console.log(dijkstra);    
    var shortestPath = dijkstra(M.g, a);
    console.log(shortestPath);
    var S = { s: [], cur: cur, fixed: {}, perturbed: {} , dsp: shortestPath};
    var num = 1;
    
    S = this.perturb(d,M,C,S,a,v,num);
    num++;
    while (S.s.length > 0) {
	console.log("pre-relieve",S.cur);
	S = this.relieves(d,M,S.cur,S,num);
	console.log("post-relieve",S.cur);
	num++;
    }
    console.log("XXXX");
    console.log(S);
    return S.cur;
}

const ANO = require("./actnetopt.js");
