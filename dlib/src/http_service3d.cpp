/*
 * Note: This is now modified to access the actnetopt optimizer.
 * Copyright (C) Robert L. Read, 2018, license MIT or GPL3 (not sure which!)
 * Example illustrating HTTP service.
 *
 * Server Usage:
 *    sudo ./distribution/example/http_service
 *
 * Client Usage:
 *    curl -w'\n' -v -X POST --data 'Hello, Restbed' 'http://localhost/goal'
 */

#include <memory>
#include <cstdlib>
#include <restbed>
#include "json.hpp"
#include "Tetrahelix.hpp"

// #include "Obstacle.hpp"
#include "playground3d.hpp"
#include <dlib/optimization.h>
#include "custom_logger.hpp"


using namespace std;
using namespace restbed;



// for convenience
using json = nlohmann::json;

Tetrahelix *an;
  
column_vector* coordsx;


void get_method_handler_goal( const shared_ptr< Session > session )
{
    cerr << "goal invoked\n";
  
    const auto request = session->get_request( );

    size_t content_length = request->get_header( "Content-Length", 0 );

    session->fetch( content_length, [ request ]( const shared_ptr< Session > session, const Bytes & body )
    {

      auto locked_nodes = request->get_query_parameter( "locked" );

      cout << "LOCKED" << locked_nodes << "\n";
      
      auto j3 = json::parse(locked_nodes);

      // special iterator member functions for objects
      int i = 0; 
      for (json::iterator it = j3.begin(); it != j3.end(); ++it) {
	std::cout << i++ << "\n";
	std::cout << it.key() << " : " << it.value() << "\n";
	auto v = it.value();
	// find an entry
	if (v.find("node_number") != v.end()) {
	  // there is an entry with key "node_number
	  auto n = v.find("node_number");
	  int num = *n;
	  cout << "Node Number found to be: " << num << "\n";
	  auto pos = v.find("pos");
	  auto p = *pos;
	  double x = *p.find("x");
	  double y = *p.find("y");
	  double z = *p.find("z");
	  cout << " x y z " << x << " " << y << " " << z << "\n";
	    column_vector gl(3);
	    gl(0) = x;
	    gl(1) = y;
	    gl(2) = z;
	    if (i < an->goals.size()) {
	      an->goals[i] = gl;
	      an->goal_nodes[i] = num;
	    } else {
	      an->add_goal_node(num,gl(0),gl(1),gl(2),1.0);        	      
	    }

	}
      }

      
      // cout << "XXX" << request->get_query_parameter( "x" ) << "\n";
      // cout << "YYY" << request->get_query_parameter( "y" ) << "\n";
      // cout << "ZZZ" << request->get_query_parameter( "z" ) << "\n";      
      
      // double x = request->get_query_parameter( "x",10.0 );
      // double y = request->get_query_parameter( "y",10.0 );
      // double z = request->get_query_parameter( "z",10.0 );

      // int num = 1;
      
      if (true) {
	  // there is an entry with key "foo"
	  // cout << "FOUND X AND Y and Z!\n";
	  // for(int i = 0; i < num; i++ ) {
	  //   column_vector gl(3);
	  //   gl(0) = x;
	  //   gl(1) = y;
	  //   gl(2) = z;
	  //   an->goals[an->goals.size() - 1] = gl;
	  // }

	    
    	  //	  handle_goal_target_physical(an,coordsx,x,y,z);
	  handle_goal_target_physical(an,coordsx);	
	  json solution;
	  for(int i = 0; i < an->num_nodes; i++) {
	    std::cout << " d["<< i << "]" << coordsx[i](0) << "," << coordsx[i](1) << "," << coordsx[i](2) << std::endl;
	    solution[i]["x"] = coordsx[i](0);
	    solution[i]["y"] = coordsx[i](1);
	    solution[i]["z"] = coordsx[i](2);	    	    
	  }

	  std::string s = solution.dump(4);
	  fprintf( stdout, "%s\n", s.c_str() );
	  std::ostringstream oss;
	  oss << s.length();
	  std::string len = oss.str();
	  session->close( OK, s.c_str(), {  { "Content-Length", len } } );	  
	} else {
	  session->close( BAD_REQUEST, "could not find x or y", { { "Content-Length", "21" }, { "Connection", "close" } } );	  
	}
	
	//	fprintf( stdout, "%.*s\n", ( int ) body.size( ), body.data( ) );
	// output the solution as JSON!

    } );
}

void options_method_handler_goal( const shared_ptr< Session > session )
{
    const auto request = session->get_request( );

	  session->close( OK, "SUCCESS", {
	      {"Access-Control-Allow-Headers", "*" 
		  } } );	  
}

int main( const int, const char** )
{
    auto resource = make_shared< Resource >( );
    resource->set_path( "/goal" );
    resource->set_method_handler( "GET", get_method_handler_goal );
    // This is to deal with the CORS stuff by sending back "Access-Control-Allow-Headers"
    resource->set_method_handler( "OPTIONS", options_method_handler_goal );

    auto settings = make_shared< Settings >( );
    //    settings->set_port( 1984 );
    //    settings->set_default_header( "Connection", "close" );
    settings->set_default_header( "Access-Control-Allow-Origin", "*" );
    cout << "TRUSS_NODES : " << TRUSS_NODES << "\n";

    an = init_Tetrahelix(TRUSS_NODES,UPPER_BOUND,LOWER_BOUND,INITIAL);
    coordsx = new column_vector[an->num_nodes];


    mainx(an,coordsx);

    Service service;
    service.publish( resource );
    service.set_logger( make_shared< CustomLogger >( ) );
    service.start(settings );

    cerr << "Service Invoked!\n";

    return EXIT_SUCCESS;
}
