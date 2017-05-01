
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

var g = require('algorithms').DataStructures;

module.exports.dim2 = 2;


// Let's see if we can just do a good job computing
// a score from a model 
module.exports.score = function(coords,goals) {
    var sum = goals.map(g =>  {
	var c = coords[g.nd];
	var q = g.pos.distanceToSquared(c);
	return q * g.wt;
    }).reduce(function(acc,elem) {
	return acc+elem;
    },0);
    return sum;
}

module.exports.opt = function(dim,model,goals,fixed) {
    return true;
}
