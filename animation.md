---
layout: default
title: Actuator Net Optimization
---
    
    
     
<div id="content-wrapper">
      <div class="inner clearfix">
        <section id="main-content">
    <section id="visualsection" style="{border: red;}" class="xscrollable">
    </section>

<p>
    <label for="animation_val">Animation Slider</label>
  <input type="text" id="animation_val" readonly style="border:0; color:#f6931f; font-weight:bold;">
    <div id="animation_pos"></div>
</p>    

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


// We need to decide if this constains an array of points
// or an object model.  I think the object model is better (that is, and object named by nodeds.)
// So I will now repair that...
var ALL_SOLUTIONS = [];



var TARGET_X = 0;
var TARGET_Y = 0;


// Make an instance of two and place it on the page.
var elem = document.getElementById('visualsection');


var params = { width: 1000, height: 1000 };
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
//    y = y + params.height / (2 *(2 * h));
//    x = x + params.width / (2 * (2 * w)) ;
    return [x,y];
}

function transform_from_viewport(x,y) {

        // now move to origin...
    x = x - (params.width)/2;
    y = y - (params.height)/2;

    
    // then unscale..
    x = x / (params.width / (2*w));
    y = - y / (params.height / (2*h));    


    
    return [x,y];
}

function test_transforms() {
    for(var x = -10; x < 10; x++) {
	for(var y = -10; y < 10; y++) {
	    var p = transform_from_viewport(x,y);
	    var v = new THREE.Vector2(p[0],p[1]);
	    var r = transform_to_viewport(v);
	    console.log(x,y);
	    console.log(r);
	    
//	    assert(r.x == x);
//	    assert(r.y == y);
	}
    }
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
    if (C) {
	for(var c in C) {
//    Object.keys(C).forEach(c => {
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
    };
    var es = edges(M.g);
    es.forEach(e =>
	       {
		   var pnt0 = C[e[0]];
		   var pnt1 = C[e[1]];
		   var tpnt0 = transform_to_viewport(trans(pnt0));
		   var tpnt1 = transform_to_viewport(trans(pnt1));

		   var line = two.makeLine(tpnt0[0], tpnt0[1],tpnt1[0], tpnt1[1]);
		   line.fill = '#FF0000';
		   line.stroke = color; // Accepts all valid css color
		   line.linewidth = 3;
	       });
    }
}

createGrid(params.width / (2 * 10.0));
render_origin();

var stm = ANO.simple_triangle_problem();

var NODE = stm.goals[0].nd;
console.log("GOAL",NODE);

stm.goals[0] = { nd: NODE,
	     pos: new THREE.Vector2(2,5),
		 wt: 3 };



function render_origin() {
    var origin = transform_to_viewport(new THREE.Vector2(0,0));
    var circle = two.makeCircle(origin[0], origin[1], 2);
    circle.fill = '#000000';
    circle.stroke = 'black'; // Accepts all valid css color
    circle.linewidth = 2;
}

function render_spot(x,y) {
    var pnt = transform_to_viewport(new THREE.Vector2(x,y));
    var circle = two.makeCircle(pnt[0], pnt[1], 3);
    circle.fill = 'blue';
    circle.stroke = 'blue'; // Accepts all valid css color
    circle.linewidth = 3;
}

function translate(C,v) {
    return Object.keys(C).forEach(x => C[x] = C[x].add(v));
}


render_origin();



var targ = new THREE.Vector2(0,4);
var origin = transform_to_viewport(new THREE.Vector2(0,0));

const cur = {};

var C = stm.coords;

Object.keys(C).forEach( nd => cur[nd] = ANO.copy_vector(C[nd]));



var shortestPath = ANO.dijkstra(stm.model.g, NODE);

var S = { s: [], used: [], cur: cur, fixed: {}, perturbed: {} , dsp: shortestPath};

var ALGS_PER_ROW = 4;
var step = 0;


console.log("S.cur",S.cur);

var N = 2;

// In order to be able to understand this, I want to store an array of solutions
// so that I can animate them after the fact!

var step = 0;
var color = ['red','blue','green','purple','gray'];
var ycnt = 0;
var xcnt = 0;
var limit = 400;

step = 0;

// var D = ANO.strainfront(stm.d,stm.model,C,'d',new THREE.Vector2(4,4),animate);
// console.log("D = ",D);

var lstep = 0;

function animate(s) {
    two.clear();
	createGrid(params.width / (2 * 10.0));
	render_origin();
    //    console.log(s);
    var ALGS_PER_ROW = 6;
//    if ((step % 2) == 0) {
    //
//    var lstep = step / 2;
//    xcnt = lstep  % ALGS_PER_ROW;
//	ycnt = Math.floor(lstep / ALGS_PER_ROW);

	xcnt = 2;
	ycnt = 1;

	var c = {};
	for(var k in s.cur) {
	    c[k] = ANO.copy_vector(s.cur[k]);
	}
	ALL_SOLUTIONS.push(c);	    

	render_graph(stm.model,s.cur,
		     color[lstep % color.length],
		     ( x => {
			 var p = ANO.copy_vector(x);
			 p.add(new THREE.Vector2(0,0));
//			 p.add(new THREE.Vector2((xcnt-(ALGS_PER_ROW/2))*6,10*((-ycnt+1))));
			 return p;
		     }));
  //  }
	step++;
    console.log(step,s.s.length);
    console.log(step,s.s);    

    //    alert();
    two.update();
    return (step < limit);
}

render_one = function(C,x,y,xcnt,ycnt) {
	two.clear();
	createGrid(params.width / (2 * 10.0));
	render_origin();
	// I don't know why I have to do this!!
//	var p = transform_from_viewport(x,y);
//	var x = p[0];
//	var y = p[1];
         render_spot(x,y);
	    render_graph(stm.model,C,
			 color[lstep % color.length],
			 ( x => {
			     var p = ANO.copy_vector(x);
			     p.add(new THREE.Vector2((xcnt-(ALGS_PER_ROW/2))*6,10*((-ycnt+1))));
			     return p;
			 })
			);
    two.update();

    steps = 0;
}

var MAX_STEP_NUM = 500;
function do_one_numerical_optimization(nd,x,y) {
    TARGET_X = x;
    TARGET_Y = y;
//    var params = {'maxIterations' : 5, 'history' : []};

//    var stm = ANO.medium_triangle_problem();
    var om = ANO.construct_optimization_model_from_ANO(stm);

	stm.goals[0] = { nd: nd,
			 pos: new THREE.Vector2(x,y),
			 wt: 3 };
    
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
    ALL_SOLUTIONS = ALL_SOLUTIONS.slice(0,0);

    var fvn = function(X,n) {
	var fxprime = Array.apply(null, Array(X.length)).map(Number.prototype.valueOf,0);

	return ANO.f(X,fxprime,stm,Y,Z,names,V,F)[n];	    
    }

    var fv = function(X) {
	return fvn(X,0);
    }
    var fd = function(X) {
	if ((step % N) == 0) {
	    var lstep = step / N;

	    xcnt = 2;
	    ycnt = 1;

	    
	    var C = {};
	    Object.keys(stm.fixed).forEach(function (n,ix) {
		var k = Object.keys(stm.fixed)[ix];
		C[k] = new THREE.Vector2(stm.coords[n].x,stm.coords[n].y);
	    });
	    names.forEach(function (n,ix) {
		var k = names[ix];
		C[k] = new THREE.Vector2(X[ix*2],X[ix*2+1]);
	    });

	    ALL_SOLUTIONS.push(C);	    
	    render_one(C,x,y,xcnt,ycnt,color[lstep % color.length]);

	}
	step++;
	if (step > MAX_STEP_NUM) {
	    throw new Error("MAX STEPS EXCEEDED!");
	}
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

    console.log("MAX LEN", ALL_SOLUTIONS.length);
    $( "#animation_pos" ).slider( "option", "max", ALL_SOLUTIONS.length );

    console.log("C",C);
    console.log(ANO.max_non_compliant(stm.model,C));
    var mc = ANO.max_non_compliant(stm.model,C);
}

function do_one_strainfront_optimization(nd,x,y) {
    TARGET_X = x;
    TARGET_Y = y;
    stm.goals[0] = { nd: nd,
			 pos: new THREE.Vector2(x,y),
			 wt: 3 };
    

    console.log(stm);
    ALL_SOLUTIONS = ALL_SOLUTIONS.slice(0,0);

    var D = ANO.strainfront(stm.dim,stm.model,stm.coords,nd,stm.goals[0].pos,animate);
    console.log("D = ",D);
    console.log("Penalty:",ANO.max_non_compliant(stm.model,D));

    console.log("MAX LEN", ALL_SOLUTIONS.length-1);
    $( "#animation_pos" ).slider( "option", "max", ALL_SOLUTIONS.length-1 );

}

var USE_STRAINFRONT = 0;
;
$("#visualsection")
    .bind('click', function(e) {
	two.clear();

	ycnt = 0;
	xcnt = 0;
	step = 0;
	
	createGrid(params.width / (2 * 10.0));
	render_origin();
	// I don't know why I have to do this!!
	var p = transform_from_viewport(e.offsetX,e.offsetY);
	var x = p[0];
	var y = p[1];
	render_spot(x,y);
	console.log("QQQQ",x,y);
	steps = 0;
	if (USE_STRAINFRONT) {
	    do_one_strainfront_optimization(NODE,x,y);
	} else {
	    do_one_numerical_optimization(NODE,x,y);
	}
	console.log("NON_COMPLIANCE",ANO.max_non_compliant(stm.model,ALL_SOLUTIONS[ALL_SOLUTIONS.length-1]));
	});


// render_loop();

$(window).ready( function() {
    $( "#animation_pos" ).slider(
       	{
	    range: "max",
	    min: 0,
	    max: 80,
	    value: 0,
	    change: function( event, ui ) {
		var n = ui.value;
		console.log("n =", n);
       		$( "#animation_val" ).val( n );
		var sol = ALL_SOLUTIONS[n];
		console.log("sol",sol);
		// This needs to be regularized to be the center in some way
		if (sol) {
		xcnt = 2;
		ycnt = 1;
		render_one(sol,TARGET_X,TARGET_Y,xcnt,ycnt,color[n % color.length]);
		    console.log("NON_COMPLIANCE",ANO.max_non_compliant(stm.model,ALL_SOLUTIONS[n]));
		}
	    }

       	}
    );
    $( "#animation_val" ).val( $( "#animation_pos" ).slider( "value" ) );
  });

    </script>
