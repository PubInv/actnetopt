
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


var THREE = require('./javascripts/three.js');
var assert = require('assert');

// Let's see if we can just do a good job computing
// a score from a model

module.exports.dim2 = 2;

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

// compute the nodes that limit our ability to reach
// the goal point and return those that limit and
// the parametric travel on the interval [0..1] in
// the direction vector, where those with 1 need not
// be returned. If not node is a limit, then
// it returns an empty list.
module.exports.limits = function(model,cur,cg) {
    var nam = cg.nd;
    var pos = cur[nam];
    // TODO: This is dim-dependent.
    var dir = this.copy_vector(cg.pos);
    dir.subVectors(dir,pos);

    var limits = [];
    // now we need to iterate over only the connected nodes from model...
    var neighbors = model.g.neighbors(nam);
    neighbors.forEach(neighbor => {
	var ename = (nam < neighbor) ? nam + ' ' + neighbor
	    : neighbor + ' ' + nam;
	var lb = model.lbs[ename];
	var ub = model.ubs[ename];
	var npos = cur[neighbor];
	

	var d = npos.distanceTo(cg.pos);
	
	var ndir = this.copy_vector(cg.pos);
	ndir.subVectors(ndir,npos);

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
	} else if (d > ub) {
	    var delta = d - ub;
	    var new_goal = this.copy_vector(npos);
	    ndir.setLength(ub);
	    new_goal.add(ndir)
	    var dtm = new_goal.distanceTo(pos);
	    
	    limits.push([neighbor,dtm,new_goal]);	    
	} else {
	    // we're okay!
	}
    }
		     );
    return limits;
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

module.exports.opt = function(dim,model,coords,goals,fixed) {
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
		      pq.insert(g.nd, [0,-this.score1(coords,g)])
		  });
    var cnt = 0; 

    // now begin the interative processing...
    var cnam = pq.extract();
    var cg = goals.find(g => g.nd == cnam);
    while (cg) {
	// Now cg is the "worst" goal we need to try to move...
	// compute the direction to move...
	var nam = cg.nd;
	var pos = cur[nam];
	var dir = this.copy_vector(pos);
	dir.subVectors(dir,pos)
	// Now we want to try to move in this direction until
	// we hit a constaint..
	var limiting_nodes = this.limits(model,cur,cg,dir);

	if (limiting_nodes.length > 0) {
	    // Now compute the maximum move the limits allow
	    // (with 1.0 in case of empty...
	    var min_move = this.min_move(limiting_nodes,pos);

	    // now min_move is a triple  chosen from limits....

	    if (min_move[1] == 0.0) {
		// In this case, there is no point n moving, we must move one of
		// our neighbors (this one) in an attempt to get closer if we can.
		// This is effective this point where we need to recurse or do something different.
		// Now if we can't get min_mov[0] to move, we are done.
		
		
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
	    // If there are not limits, we can move directly to the goal...
	    // and we need add nothing to the pqueue...
	    cur[nam] = this.copy_vector(cg.pos);
	}
	var cnam = pq.extract();
	cg = goals.find(g => g.nd == cnam);
	cnt++;
    }
    return cur;
}


