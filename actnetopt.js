
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

var THREE = require('./javascripts/three.js');
var assert = require('assert');

module.exports.dim2 = 2;


// Let's see if we can just do a good job computing
// a score from a model

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
    console.log("dir",dir);
    dir.subVectors(dir,pos);

    var limits = [];
    // now we need to iterate over only the connected nodes from model...
    var neighbors = model.g.neighbors(nam);
    neighbors.forEach(neighbor => {
	var lb = model.lbs[neighbor];
	var ub = model.ubs[neighbor];
	var npos = cur[neighbor];
	// actually, I think there is a linear relationship
	// between the parameter length and this...but
	// for right now, I'll just treat it as limited or not.
	var d = npos.distanceTo(cg.pos);

	// TODO: this should be replacable
	if (d < lb) {
	    var delta = lb - d;
	    var new_goal = this.copy_vector(npos);
	    console.log("lower goal",new_goal);
	    var s = delta / dir.length();
	    new_goal.addScaledVector(dir,-s);
	    console.log("lower goal X",s,new_goal);	    	    
	    limits.push([neighbor,-s,new_goal]);	    
	} else if (d > ub) {
	    var delta = d - ub;
	    var new_goal = this.copy_vector(npos);
	    console.log("upper goal",new_goal,dir.length());	    
	    var s = delta / dir.length();
	    new_goal.addScaledVector(dir,s);
	    console.log("upper goal X",s,new_goal);	    
	    limits.push([neighbor,s,new_goal]);	    
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
module.exports.max_move = function(limits,pos) {
    // find the position that is definitely inside the limits!
    var init = limits[0];
    var possible_move = limits.reduce(function(acc,val) {
	return (Math.abs(val[1]) < Math.abs(val[1])) ?
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
    var pq = new algols.PriorityQueue();

    // now it is possible that this can add a
    // zero goal...we will have to be careful of that.
    // Really I need to have my own comparator here that
    // orders by greated weighted distance first, then
    // by shortest move.
    goals.forEach(g => pq.insert(g, -this.score1(coords,g)));
    var cnt = 0; 

    // now begin the interative processing...
    var cg = pq.extract();
    while (cg && cnt < 4) {
	// Now cg is the "worst" goal we need to try to move...
	// compute the direction to move...
	var nam = cg.nd;
	var pos = cur[nam];
	var dir = this.copy_vector(pos);
	console.log("dirx",dir);		
	dir.subVectors(dir,pos)
	// Now we want to try to move in this direction until
	// we hit a constaint..
	var limiting_nodes = this.limits(model,cur,cg,dir);
	console.log("max_move",limiting_nodes);	
	if (limiting_nodes.length > 0) {
	    // Now compute the maximum move the limits allow
	    // (with 1.0 in case of empty...
	    var max_move = this.max_move(limiting_nodes,pos);

	    // now max_move is a triple  chosen from limits....
	    assert(max_move[1] != 0.0);

	    // make the move...
	    cur[nam] = this.copy_vector(max_move[2]);
	    // rebuild the pq...
	    pq = new algols.PriorityQueue();

	    goals.forEach(g => pq.insert(g, -this.score1(cur,g)));	    
	    // if we moved, then basically we start all over with all
	    // computations...
	} else {
	    // if we couldn't move, then we need to add some goals
	    // to our priority_queue based on limits...
	}
	console.log(pq);
	cg = pq.extract();
	cnt++;
    }
    return cur;
}

