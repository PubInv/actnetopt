---
layout: default
title: Actuator Net Optimization
---
    
    
     
<div id="content-wrapper">
      <div class="inner clearfix">
        <section id="main-content">
    <section id="visualsection" style="{border: red;}">
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

    const ANO = UGLY_GLOBAL_SINCE_I_CANT_GET_MY_MODULE_INTO_THE_BROWSER;

// Make an instance of two and place it on the page.
var elem = document.getElementById('visualsection');


var params = { width: 600, height: 600 };
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


// Input is a THREE.Vector2, out put an [x,y] array...
function transform_to_viewport(pnt) {

    // Let's assume our play space is from -10 to + 10, centered on the origin...
    var w = 10.0;
    var h = 10.0;
    var x = pnt.x;
    var y = pnt.y;
    // first scale appropriately
    x *= x * (params.width / (2 * w));
    y *= y * (params.height / (2 * h));    
    // now move to origin....
    x += params.width/2;
    y = (-y) + params.height/2;

    // These adjust our weird grid background to the origin...
    y = y + params.width / (2 *(2 * 10.0));
    x = x + params.width / (2 * (2 * 10.0)) ;
    return [x,y];
}

// return a list of all edges in the graph....
function edges(g) {
    var edges = [];
    g.vertices.
	forEach(v =>
		{
		    console.log(v);		    
		    console.log(g.adjList[v]);
		    Object.keys(g.adjList[v]).
			forEach(av =>
				{ if (v < av) edges.push([v,av]); 
			       }
			       )
		}
	       );
    return edges;
}
function render_problem(p) {

    Object.keys(p.coords).forEach(c => {

	var pnt = p.coords[c];
	var tpnt = transform_to_viewport(pnt);
	var circle = two.makeCircle(tpnt[0], tpnt[1], 4);
	circle.fill = '#FF0000';
	circle.stroke = 'red'; // Accepts all valid css color
	circle.linewidth = 3;
    });
    var es = edges(p.model.g);
    es.forEach(e =>
	       {
		   var pnt0 = p.coords[e[0]];
		   var pnt1 = p.coords[e[1]];
		   var tpnt0 = transform_to_viewport(pnt0);
		   var tpnt1 = transform_to_viewport(pnt1);
		   
		   var line = two.makeLine(tpnt0[0], tpnt0[1],tpnt1[0], tpnt1[1]);
		   line.fill = '#FF0000';
		   line.stroke = 'red'; // Accepts all valid css color
		   line.linewidth = 3;
	       });
}

createGrid(params.width / (2 * 10.0));



var stp = ANO.medium_triangle_problem();

function render_origin() {
var origin = transform_to_viewport(new THREE.Vector2(0,0));
var circle = two.makeCircle(origin[0], origin[1], 2);
console.log(origin);
circle.fill = '#000000';
circle.stroke = 'black'; // Accepts all valid css color
circle.linewidth = 2;
}


render_problem(stp);
render_origin();

two.update();

    </script>

  

