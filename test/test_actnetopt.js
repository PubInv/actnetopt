'use strict';
var assert = require('assert');

var ANO = require('../actnetopt.js');
var algols = require('algorithms').DataStructures;
var THREE = require('../javascripts/three.js');


describe('actoptnet_math', function() {
    describe('first_test', function() {
    	it('has a variable that is true', function() {
    	    assert.equal(ANO.dim2,2);
    	});
    });
    describe('main_algorithm', function() {
    	it('function exists and returns something', function() {
    	    var stm = ANO.simple_triangle_problem();

    	    var result = ANO.opt(stm.dim2,
    				 stm.model,
    				 stm.coords,
    				 stm.goals);

    	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);	    
    	});
    });
    describe('main_algorithm', function() {
    	it('we can do something with a 2d triangle', function() {
    	    var dim = ANO.dim2;
    	    var model = new algols.Graph();
    	    model.addVertex(1);
    	    model.addVertex(2);
    	    model.addVertex(3);
    	    model.addEdge(1, 2);
    	    model.addEdge(2, 3);
    	    model.addEdge(1, 3);
    	    assert.equal(model.vertices.size,3);
    	});
    });
    // describe('limits', function() {
    // 	it('limits returns a useful answer', function() {
    // 	    var stm = ANO.simple_triangle_problem();
    // 	    var limits = ANO.limits(stm.model,stm.coords,stm.goals[0]);
    // 	    assert.equal(limits.length,0);
    // 	});
    // });
    describe('score', function() {
    	it('we can compute a score', function() {
    	    var stm = ANO.simple_triangle_problem();
    	    var sc = ANO.score(stm.coords, stm.goals);
    	    assert.equal(sc,54);
    	});
    });
    describe('main_algorithm', function() {
    	it('we can invoke it', function() {

    	    var stm = ANO.simple_triangle_problem();

    	    var result = ANO.opt(stm.dim2,
    				 stm.model,
    				 stm.coords,
    				 stm.goals);
	    
    	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);
    	});
    });
    describe('main_algorithm', function() {
    	it('We get a good answer for a siimple triangle', function() {
    	    var stm = ANO.simple_triangle_problem();

    	    var result = ANO.opt(stm.dim2,
    				 stm.model,
    				 stm.coords,
    				 stm.goals);
    	    var score = ANO.score(result,stm.goals);


    	    assert.equal(ANO.legal_configp(stm.model,result),true);

    	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);

    	    assert.ok(score < 40.2);
    	});
    });
    // describe('main_algorithm', function() {
    // 	it('We can optimize two goals if simple', function() {
    // 	    var stm = ANO.simple_triangle_problem();

    // 	    // remove the 'b' note from fixed
    // 	    stm.fixed.slice(0,2);
    // 	    stm.goals[1] = { nd: 'b',
    // 		 pos: new THREE.Vector2(0.3,1.7),
    // 		 wt: 3 };

    // 	    var result = ANO.opt(stm.dim2,
    // 				 stm.model,
    // 				 stm.coords,
    // 				 stm.goals);
    // 	    var score = ANO.score(result,stm.goals);
    // 	    console.log("result,score =",result,score);

    // 	    assert.equal(ANO.legal_configp(stm.model,result),true);

    // 	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);

    // 	    // assert that b and c both moved
    // 	    assert.deepEqual(result['a'],stm.coords['a']);	    
    // 	    assert.notDeepEqual(result['b'],stm.coords['b']);
    // 	    assert.notDeepEqual(result['c'],stm.coords['c']);	    
    // 	    assert.ok(score < 40.6);
    // 	});
    // });
    // describe('main_algorithm', function() {
    // 	it('We can deal with a more complex function', function() {
    // 	    var stm = ANO.medium_triangle_problem();
    // 	    assert.equal(ANO.legal_configp(stm.model,stm.coords),true);
	    
    // 	    var result = ANO.opt(stm.dim2,
    // 				 stm.model,
    // 				 stm.coords,
    // 				 stm.goals);
    // 	    var score = ANO.score(result,stm.goals);
    // 	    console.log(result,score);

    // 	    assert.equal(ANO.legal_configp(stm.model,result),true);

    // 	    assert.equal(Object.keys(result).length, Object.keys(stm.coords).length);
    // 	    assert(score < 20);
    // 	});
    // });
});

describe('strainfront_algorithm', function() {
    // describe('first_test', function() {
    // 	it('has a variable that is true', function() {
    // 	    assert.equal(ANO.dim2,2);
    // 	});
    // });

    // describe('circle math', function() {
    // 	it('can compute circle intersctions', function() {
    // 	    var v0 = new THREE.Vector2(0,0);
    // 	    var v1 = new THREE.Vector2(1,0);
    // 	    var r0 = 0.0;
    // 	    var r1 = 0.1;
    // 	    var res0 = ANO.circle_intersections(v0,r0,v1,r1);
    // 	    assert(res0.length == 0);
    // 	    var r0 = 0.5;
    // 	    var r1 = 0.5;
    // 	    var res0 = ANO.circle_intersections(v0,r0,v1,r1);
    // 	    assert(res0.length == 1,res0);
    // 	    var r0 = 0.6;
    // 	    var r1 = 0.6;
    // 	    var res0 = ANO.circle_intersections(v0,r0,v1,r1);	    
    // 	    assert(res0.length == 2);
    // 	});
    // });

    // describe('STRAIN', function() {
    // 	it('We can compute strain', function() {
    // 	    var stm = ANO.simple_triangle_problem();
    // 	    var s0 = ANO.strain(stm.dim2,
    // 				stm.model,
    // 				stm.coords,
    // 				'a',
    // 				'c');
    // 	    assert.equal(s0,0);
    // 	    // This should produce tensile strain
    // 	    stm.coords['c'] = new THREE.Vector2(4,4);
    // 	    var s1 = ANO.strain(stm.dim2,
    // 				stm.model,
    // 				stm.coords,
    // 				'a',
    // 				'c');
    // 	    assert(s1 < 0);
	    
    // 	    // This should produce compressive strain
    // 	    stm.coords['c'] = new THREE.Vector2(0,0);
    // 	    var s2 = ANO.strain(stm.dim2,
    // 				stm.model,
    // 				stm.coords,
    // 				'a',
    // 				'c');
    // 	    assert(s2 > 0);	    
    // 	});
    // });

    // describe('bound_intersections', function() {
    // 	it('gives basic resulsts', function() {
    // 	    var stm = ANO.simple_triangle_problem();
    // 	    stm.coords['a'] = new THREE.Vector2(0,0);
    // 	    stm.coords['c'] = new THREE.Vector2(3,0);	    
    // 	    var s0 = ANO.bound_intersections(stm.dim2,
    // 				stm.model,
    // 				stm.coords,
    // 				'a',
    // 					     'c');
    // 	    assert.equal(4,s0.length,s0);
    // 	    // This should produce tensile strain
    // 	    stm.coords['c'] = new THREE.Vector2(0,1.5);
    // 	    var s1 = ANO.bound_intersections(stm.dim2,
    // 				stm.model,
    // 				stm.coords,
    // 				'a',
    // 				'c');
    // 	    assert.equal(8,s1.length);
	    
    // 	    // This should produce compressive strain
    // 	    stm.coords['c'] = new THREE.Vector2(0,0);
    // 	    var s2 = ANO.bound_intersections(stm.dim2,
    // 				stm.model,
    // 				stm.coords,
    // 				'a',
    // 				'c');
    // 	    assert(s2);	    
    // 	});
    // });
    // describe('all_strains', function() {
    // 	it('all_strains increases as we get beyond upper bound', function() {
    // 	    var stm = ANO.simple_triangle_problem();
    // 	    for(var i = 0; i < 10; i++) {
    // 		console.log(ANO.all_strains(stm.d,stm.model,stm.coords,'c',new THREE.Vector2(0,i)));
    // 	    }
    // 	});
    // });

    // describe('ZERO_X_STRAIN', function() {
    // 	it('computes a position with zero strain for a super-liberal bound set', function() {
    // 	    var stm = ANO.simple_triangle_problem();
    // 	    // We set up a very simple situation and show that ZERO_X_STRAIN
    // 	    // minimizes the x->y strain
    // 	    const lb = 0;
    // 	    const ub = 4;
    // 	    const m = stm.model
    // 	    m.lbs['a b'] = lb;
    // 	    m.ubs['a b'] = ub;
    // 	    m.lbs['b c'] = lb;
    // 	    m.ubs['b c'] = ub;
    // 	    m.lbs['a c'] = lb;
    // 	    m.ubs['a c'] = ub;
	    
    // 	    var NUM = 4;
    // 	    for(var i = 0; i < NUM; i++) {
    // 		for(var j = 0; j < NUM; j++) {
    // 		    stm.coords['c'] = new THREE.Vector2(i,j);
    // 		    var z = ANO.zero_x_strain(stm.dim2,stm.model,stm.coords,'a','c');
    // 		    console.log(i,j,z);
    // 		    stm.coords['c'] = z;
    // 		    assert(ANO.strain(stm.dim2,stm.model,stm.coords,'a','c') == 0);		    
    // 		}
    // 	    }
    // 	});
    // 	it('computes a position with zero strain for an integer grid', function() {
    // 	    var stm = ANO.simple_triangle_problem();
    // 	    // We set up a very simple situation and show that ZERO_X_STRAIN
    // 	    // minimizes the x->y strain
    // 	    var NUM = 5;
    // 	    for(var i = 3; i < NUM; i++) {
    // 		for(var j = 3; j < NUM; j++) {
    // 		    stm.coords['a'] = new THREE.Vector2(i,j);
    // 		    var z = ANO.zero_x_strain(stm.dim2,stm.model,stm.coords,'a','c');
    // 		    console.log(i,j,z);
    // 		    stm.coords['c'] = z;
    // 		    console.log(ANO.strain(stm.dim2,stm.model,stm.coords,'a','c'));
    // 		    assert(ANO.strain(stm.dim2,stm.model,stm.coords,'a','c') == 0);
    // 		}
    // 	    }
    // 	});
    // });
    // describe('max_strain_on_point', function() {    
    //     it('strain is correctly compute simple values', function() {
    // 	    var stm = ANO.simple_triangle_problem();
    // 	    var z = ANO.max_strain_on_point(stm.dim2,stm.model,stm.coords,'c',new THREE.Vector2(0,2));
    // 	    assert.equal(0.5,z);
    // 	    var z = ANO.max_strain_on_point(stm.dim2,stm.model,stm.coords,'c',new THREE.Vector2(0,3));
    // 	    assert.equal(1,z);
    // 	    var z = ANO.max_strain_on_point(stm.dim2,stm.model,stm.coords,'c',new THREE.Vector2(1,3));
    // 	    assert(z > 1);
    // 	});
    //     it('strain is correctly computes a matrix', function() {
    // 	    var stm = ANO.simple_triangle_problem();
    
    // 	    var NUM = 5;
    // 	    for(var i = 0; i < NUM; i++) {
    // 		for(var j = 0; j < NUM; j++) {
    // 		    var z = ANO.max_strain_on_point(stm.dim2,stm.model,stm.coords,'c',new THREE.Vector2(i,j));
    // 		    console.log(i,j,z);
    // 		}
    // 	    }
    // 		});
    // });

    describe('STRAIN_FRONT', function() {
    	it('given a basic problem, returns something without crashing', function() {
	    var stm = ANO.simple_triangle_problem();
	    var NUM = 4;
	    for(var i = 3; i < NUM; i++) {
		for(var j = 3; j < NUM; j++) {
		    var C = ANO.strainfront(stm.d,stm.model,stm.coords,'c',new THREE.Vector2(i,j));
		    assert(C);
		}
	    }
	});
    	it('strainfront actually computes a solution to a simple problem', function() {
	    var stm = ANO.simple_triangle_problem();
	    console.log("coords",stm.coords);
	    var C = ANO.strainfront(stm.d,stm.model,stm.coords,'c',new THREE.Vector2(2,2));
	    console.log(C);
	    assert(C);
	    assert.equal(ANO.legal_configp(stm.model,C),true);		    
	});
    	it('Every grid point in neihborhood is reasonable', function() {
	    var stm = ANO.simple_triangle_problem();
	    var NUM = 4;
	    for(var i = 0; i < NUM; i++) {
		for(var j = 0; j < NUM; j++) {
		    var targ = new THREE.Vector2(i,j);
		    var C = ANO.strainfront(stm.d,stm.model,stm.coords,'c',new THREE.Vector2(i,j));
		    console.log(targ,C['c'],C['c'].distanceTo(targ)); 
		    assert.equal(ANO.legal_configp(stm.model,C),true);
		}
	    }
	});
    	it('Every grid point in ANO.medium, one-fixed functions',
	   function() {
	    var stm = ANO.medium_triangle_problem();
	    var NUM = 5;
	    for(var i = 0; i < NUM; i++) {
		for(var j = 0; j < NUM; j++) {
		    var targ = new THREE.Vector2(i,j);
		    var C = ANO.strainfront(stm.d,stm.model,stm.coords,'d',new THREE.Vector2(i,j));
//		    console.log(targ,C['d'],C['d'].distanceTo(targ));
//		    console.log(C); 		    

		    assert.equal(ANO.legal_configp(stm.model,C),true);
		}
	    }
	});
    	it('deals with {x:0,y:4} = d as target', function() {
	    var stm = ANO.medium_triangle_problem();
	    // TODO: This is s a problem....
	    assert.equal(ANO.legal_configp(stm.model,stm.coords),true);	    
	    var NUM = 5;
	    var targ = new THREE.Vector2(0,4);
	    var C = ANO.strainfront(stm.d,stm.model,stm.coords,'d',new THREE.Vector2(0,4));
	    console.log(targ,C['d'],C['d'].distanceTo(targ));
	    console.log(C); 		    
//	    assert.equal(ANO.legal_configp(stm.model,C),true);
	});
	
    });

});
