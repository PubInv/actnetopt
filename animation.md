---
layout: default
title: Actuator Net Optimization
---
    
    
     
<div id="content-wrapper">
      <div class="inner clearfix">
        <section id="main-content">
    <section id="visualsection" style="{border: red;}" class="xscrollable">
    </section>
    <section id="textsection" style="{border: red;}">
    <h1> Actuator Net Optimization </h1>
    This is an attempt to formulate and solve a problem that arises in the Gluss Project.
    <h2> The Problem </h2>
    There is a .pdf in the repo that attemtps to formulte the problem.

    
<h3>
<a id="authors-and-contributors" class="anchor" href="#authors-and-contributors" aria-hidden="true"><span aria-hidden="true" class="octicon octicon-link"></span></a>Authors and Contributors</h3>

    <p><a href="https://github.com/PubInv/actnetopt">Actuator Net Optimization</a> is written and maintained by

    <a href="mailto:read.robert@gmail.com">Robert L. Read</a> at <a href="https://github.com/PubInv">PubInv</a>.</p>

    <p> This is a newly created work, please send comments, ideas, or suggestions to Rob or open an issue.</p>

    <p>Check out our parent project, <a href="https://pubinv.github.io/PubInv">Public Invention</a>.</p>

    <p>
    To run the tests, execute "mocha".  To browserify, execute :
<pre>  browserify ano_shim.js -o bundle.js </pre>

        </section>

    <script src="./javascripts/two.min.js"></script>
    <script src="./javascripts/three.js"></script>    
<script src="./bundle.js"></script>    

    <script>

// Major questions:
// Why is it taking more steps than it should?
// Why isn't the last node being relaxed properly?



const ANO = UGLY_GLOBAL_SINCE_I_CANT_GET_MY_MODULE_INTO_THE_BROWSER;

// Make an instance of two and place it on the page.
var elem = document.getElementById('visualsection');


var params = { width: 1600, height: 1600 };
var two = new Two(params).appendTo(elem);

// Don't forget to tell two to render everything
// to the screen
two.update();

function createGrid(s) {

    var size = s || 30;
    var two = new Two({
	type: Two.Types.canvas,
	width: size,
	height: size
    });

    var a = two.makeLine(two.width / 2, 0, two.width / 2, two.height);
    var b = two.makeLine(0, two.height / 2, two.width, two.height / 2);
    a.stroke = b.stroke = '#6dcff6';

    two.update();

    _.defer(function() {
	$("#visualsection").css({
	    background: 'url(' + two.renderer.domElement.toDataURL('image/png') + ') 0 0 repeat',
	    backgroundSize: size + 'px ' + size + 'px'
	});
    });

}

var w = 20.0;
var h = 20.0;

// Input is a THREE.Vector2, out put an [x,y] array...
function transform_to_viewport(pnt) {

    // Let's assume our play space is from -10 to + 10, centered on the origin...

    var x = pnt.x;
    var y = pnt.y;
    // first scale appropriately
    x = x * (params.width / (2 * w));
    y = y * (params.height / (2 * h));    
    // now move to origin....
    x += params.width/2;
    y = (-y) + params.height/2;

    // These adjust our weird grid background to the origin...
    y = y + params.height / (2 *(2 * h));
    x = x + params.width / (2 * (2 * w)) ;
    return [x,y];
}

// return a list of all edges in the graph....
function edges(g) {
    var edges = [];
    g.vertices.
	forEach(v =>
		{
		    Object.keys(g.adjList[v]).
			forEach(av =>
				{ if (v < av) edges.push([v,av]); 
				}
			       )
		}
	       );
    return edges;
}

function render_graph(M,C,color,trans) {
    Object.keys(C).forEach(c => {
	var pnt = C[c];
	var tpnt = transform_to_viewport(trans(pnt));
	var circle = two.makeCircle(tpnt[0], tpnt[1], 4);
	circle.fill = '#FF0000';
	circle.stroke = 'red'; // Accepts all valid css color
	circle.linewidth = 3;
	// really this should be read from the edge is in the model..
	var lb = 1.1;
	var ub = 2.0;
	var s0 = lb * (params.width / (2 *w));
	var s1 = ub * (params.width / (2 *w));

	var circle0 = two.makeCircle(tpnt[0], tpnt[1], s0);
	circle0.stroke = 'red'; // Accepts all valid css color
	circle0.noFill();
	var circle1 = two.makeCircle(tpnt[0], tpnt[1], s1);
	circle1.stroke = 'blue'; // Accepts all valid css color
	circle1.noFill();	
    });
    var es = edges(M.g);
    es.forEach(e =>
	       {
		   var pnt0 = C[e[0]];
		   var pnt1 = C[e[1]];
		   console.log("e",e,pnt0,pnt1);
		   var tpnt0 = transform_to_viewport(trans(pnt0));
		   var tpnt1 = transform_to_viewport(trans(pnt1));

		   var line = two.makeLine(tpnt0[0], tpnt0[1],tpnt1[0], tpnt1[1]);
		   line.fill = '#FF0000';
		   line.stroke = color; // Accepts all valid css color
		   line.linewidth = 3;
		   console.log("XXX",tpnt0,tpnt1);
	       });
}

createGrid(params.width / (2 * 10.0));

var stm = ANO.medium_triangle_problem();

stm.goals[0] = { nd: 'd',
	     pos: new THREE.Vector2(3,4),
	     wt: 3 };


function render_origin() {
    var origin = transform_to_viewport(new THREE.Vector2(0,0));
    var circle = two.makeCircle(origin[0], origin[1], 2);
    console.log(origin);
    circle.fill = '#000000';
    circle.stroke = 'black'; // Accepts all valid css color
    circle.linewidth = 2;
}

function translate(C,v) {
    return Object.keys(C).forEach(x => C[x] = C[x].add(v));
}


render_origin();



var targ = new THREE.Vector2(0,4);
var step = 0;
var color = ['red','blue','green','purple','gray'];
var ycnt = 0;
var xcnt = 0;
var limit = 40;

step = 0;

// var D = ANO.strainfront(mtp.d,mtp.model,C,'d',new THREE.Vector2(4,4),animate);
// console.log("D = ",D);

var lstep = 0;
var origin = transform_to_viewport(new THREE.Vector2(0,0));

const cur = {};

var C = stm.coords;

Object.keys(C).forEach( nd => cur[nd] = ANO.copy_vector(C[nd]));

var shortestPath = ANO.dijkstra(stm.model.g, 'd');

var S = { s: [], used: [], cur: cur, fixed: {}, perturbed: {} , dsp: shortestPath};

var ALGS_PER_ROW = 4;
var step = 0;


console.log("S.cur",S.cur);

// function render_loop() {
//     for(var i = 0; i < 3; i++) {
// 	var lstep = step / 1;
// 	step++;
// 	xcnt = lstep  % ALGS_PER_ROW;
// 	ycnt = Math.floor(lstep / ALGS_PER_ROW);
	
// 	render_graph(mtp.model,S.cur,
// 	     color[lstep % color.length],
// 	     ( x => {
// 		 var p = ANO.copy_vector(x);
// 		 p.add(new THREE.Vector2((xcnt-(ALGS_PER_ROW/2))*6,10*((-ycnt+1))));
// 		 console.log("spud",p);
// 		 return p;
// 	     })
// 	    );
// 	two.update();
//     };
// }

function do_one_optimization() {
    var params = {'maxIterations' : 5, 'history' : []};

    var stm = ANO.medium_triangle_problem();
    var om = ANO.construct_optimization_model_from_ANO(stm);
    
    var F = om[0];
    var V = om[1];
    //	var X = om[2];
    var Y = om[3];
    var Z = om[4];
    var initial = om[5];
    var names = om[6];
    var fixed = om[7];
    var goal = om[8];
    var free = om[9];

    console.log(stm);
    console.log("om",om);

    var fvn = function(X,n) {
	var fxprime = Array.apply(null, Array(X.length)).map(Number.prototype.valueOf,0);
	console.log("step",step);
	if ((step % 100) == 0) {
	    var lstep = step / 100;
	    xcnt = lstep  % ALGS_PER_ROW;
	    ycnt = Math.floor(lstep / ALGS_PER_ROW);

	    var C = [];
	    Object.keys(stm.fixed).forEach(function (n,ix) {
		C[n] = new THREE.Vector2(stm.coords[n].x,stm.coords[n].y);
	    });
	    names.forEach(function (n,ix) {
		C[n] = new THREE.Vector2(X[ix*2],X[ix*2+1]);
	    });
	    console.log("C",C);
	    render_graph(stm.model,C,
			 color[lstep % color.length],
			 ( x => {
			     console.log("x =",x);
			     var p = ANO.copy_vector(x);
			     p.add(new THREE.Vector2((xcnt-(ALGS_PER_ROW/2))*6,10*((-ycnt+1))));
			     console.log("spud",p);
			     return p;
			 })
			);
	    two.update();
	}
	step++;
	return ANO.f(X,fxprime,stm,Y,Z,names,V,F)[n];	    
    }

    var fv = function(X) {
	return fvn(X,0);
    }
    var fd = function(X) {
	return fvn(X,1);
    }

    var solution = ANO.opt.minimize_L_BFGS(fv,fd,initial);

    console.log("stm",stm);
    console.log("solution",solution);
    // now for the purpose of checking, we copy our answer back into the model....
    var C = [];
    Object.keys(stm.fixed).forEach(function (n,ix) {
	C[n] = new THREE.Vector2(stm.coords[n].x,stm.coords[n].y);
    });
    names.forEach(function (n,ix) {
	C[n] = new THREE.Vector2(solution.argument[ix*2],solution.argument[ix*2+1]);
    });
    console.log("C",C);
    console.log(ANO.max_non_compliant(stm.model,C));
    var mc = ANO.max_non_compliant(stm.model,C);
}

do_one_optimization();

// render_loop();

    </script>
