# actnetopt
A formulation and solution to optimizing the positioning of linear actuator networks

## Status

The project was just begun on April 28th

## Motiation

The [Gluss Project](https://pubinv.github.io/gluss/) has constructed a robot as a network of linear actuators.
In particulary, the current glussbot has 7 tetrahedra, 10 joints, and 24 actuators.

There is a crawling gait for the robot at present. However, it is slow and awkward.  In an attempt to have good
control over the robot, we must solve a relatively abstract mathematical problems.  This repo is an attempt
to both formulate, solve, and code a solution that problem.
