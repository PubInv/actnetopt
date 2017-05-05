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
<script src="./bundle.js"></script>    

    <script>

    const ANO = UGLY_GLOBAL_SINCE_I_CANT_GET_MY_MODULE_INTO_THE_BROWSER;

// Make an instance of two and place it on the page.
var elem = document.getElementById('visualsection');


var params = { width: 600, height: 500 };
var two = new Two(params).appendTo(elem);

// two has convenience methods to create shapes.
var circle = two.makeCircle(72, 100, 50);
var rect = two.makeRectangle(213, 100, 100, 100);

// The object returned has many stylable properties:
circle.fill = '#FF8000';
circle.stroke = 'orangered'; // Accepts all valid css color
circle.linewidth = 5;

rect.fill = 'rgb(0, 200, 255)';
rect.opacity = 0.75;
rect.noStroke();

// Don't forget to tell two to render everything
// to the screen
two.update();

var stp = ANO.simple_triangle_problem();

    </script>

  

